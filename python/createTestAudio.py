import numpy as np
import librosa
import os, sys, getopt, glob

cwd = os.getcwd()
os.chdir(os.path.dirname(__file__))

# load audio
y, sr = librosa.load('testData/testAudio.mp3', sr=48000, mono=False, dtype=np.float32)

# scale to integer value
nBits = 24
y *= (2**(nBits-1) - 1)

# write data to file
fs = 48000
t = 0.1
nSamples = int(t * fs)
np.savetxt('testData/testData.h', [y[0,0:nSamples]], fmt='%i', delimiter=',\n', newline='\n', 
           header=f'#define N_SAMPLES   ((uint32_t){nSamples}) \n int32_t testData[{nSamples}] = ' + '{', footer='};', comments='', encoding=None)

# change back to original working directory
os.chdir(cwd)
