import sys
import numpy as np
import SWHear

maxFFT=0
ear = SWHear.SWHear()
ear.stream_start()


while True:
    if not ear.data is None and not ear.fft is None:
        pcmMax=np.max(np.abs(ear.data))
        if pcmMax>maxPCM:
            maxPCM=pcmMax
            #grPCM.plotItem.setRange(yRange=[-pcmMax,pcmMax])
        if np.max(ear.fft)>maxFFT:
            maxFFT=np.max(np.abs(ear.fft))
            #grFFT.plotItem.setRange(yRange=[0,maxFFT])
        #pbLevel.setValue(1000*pcmMax/maxPCM)
        #pen=pyqtgraph.mkPen(color='b')
        #grPCM.plot(ear.datax,ear.data,
        #                pen=pen,clear=True)
        #pen=pyqtgraph.mkPen(color='r')
        #grFFT.plot(ear.fftx[:500],ear.fft[:500],
        #                pen=pen,clear=True)
        print(maxFFT)
    #QtCore.QTimer.singleShot(1, update) # QUICKLY repeat

