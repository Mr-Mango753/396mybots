import numpy
import matplotlib.pyplot

backLegSensorValues = numpy.load("backLegSensorValues.npy", mmap_mode=None, allow_pickle=False, fix_imports=True, encoding='ASCII', max_header_size=1000)
frontLegSensorValues = numpy.load("frontLegSensorValues.npy", mmap_mode=None, allow_pickle=False, fix_imports=True, encoding='ASCII', max_header_size=1000)
matplotlib.pyplot.plot(backLegSensorValues, linewidth=1, label="Back Leg")
matplotlib.pyplot.plot(frontLegSensorValues, linewidth=2, label="Front Leg")
matplotlib.pyplot.legend(loc="upper right")
matplotlib.pyplot.show()
