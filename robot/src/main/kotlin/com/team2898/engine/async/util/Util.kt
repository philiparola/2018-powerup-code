package com.team2898.engine.async.util

import com.team2898.engine.async.pools.ComputePool
import com.team2898.engine.async.pools.IOPool
import kotlinx.coroutines.experimental.*
import java.util.concurrent.ForkJoinPool
import kotlin.coroutines.experimental.CoroutineContext
import kotlin.system.measureTimeMillis

fun go(
        pool: ThreadPoolDispatcher = ComputePool,
        lazy: Boolean = false,
        block: suspend () -> Unit
): Job = launch(pool, if (lazy) CoroutineStart.LAZY else CoroutineStart.DEFAULT) { block() }

fun goIO(block: suspend () -> Unit): Job = go(IOPool) { block() }
fun goLazy(block: suspend() -> Unit): Job = go(lazy=true) {block()}

class WaitGroup {
    val jobs = mutableListOf<Job>()
    fun add(job: Job): WaitGroup = this.apply { jobs.add(job) }
    fun add(block: suspend () -> Unit): WaitGroup = this.apply { jobs.add(go { block() }) }
    fun join(): Long = measureTimeMillis { runBlocking<Unit> { jobs.forEach { it.join() } } }
}

