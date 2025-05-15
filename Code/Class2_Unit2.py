import numpy
# Unit 2 Chapter 5

Jv = numpy.array([[-0.105, 0, 0.006, -0.045, 0, 0.006, 0],
                  [-0.889, 0.006, 0, -0.844, 0.006, 0, 0],
                  [0, -0.105, 0.889, 0, 0, 0, 0] ])

numpy.set_printoptions(suppress=True)
print(numpy.dot(Jv,Jv.T))

# M equals numpy.dot(Jv, Jv.T)

M = numpy.array([
    [0.013122, 0.131325, 0.005334],
    [0.131325, 1.502729, -0.00063],
    [0.005334, -0.00063, 0.801346]
])

eigenvalues, eigenvectors = numpy.linalg.eig(M)

numpy.set_printoptions(suppress=True)

print(eigenvectors)

print("------------")

print(eigenvalues)

