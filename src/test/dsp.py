import numpy as np
from scipy.signal import butter, lfilter, freqz
# import matplotlib.pyplot as plt

class Butterworth:
    def butter_lowpass(self, cutoff=5.0, fs=100, order=6):
        nyq = 0.5 * fs
        normal_cutoff = cutoff / nyq
        b, a = butter(order, normal_cutoff, btype='low', analog=False)
        return b, a

    def butter_lowpass_filter(self, data, cutoff, fs, order=5):
        b, a = self.butter_lowpass(cutoff, fs, order=order)
        y = lfilter(b, a, data)
        return y
