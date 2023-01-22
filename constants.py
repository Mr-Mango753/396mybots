import numpy


amplitude = numpy.pi/4
frequency = 5
phaseOffset = 0

amplitude_backleg = numpy.pi/4
frequency_backleg = 5
phaseOffset_backleg = numpy.pi/4

amplitude_frontleg = numpy.pi/4
frequency_frontleg = 5
phaseOffset_frontleg = 0

vectorSize = 1000
maxForce = 500

targetAngles = numpy.linspace(-numpy.pi, numpy.pi, vectorSize)