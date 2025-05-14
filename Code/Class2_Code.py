import numpy
# Chapter 2 Question 5

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


# 1 -> [0,0,1.414]

# 2 -> [30,20,10,20]

# 7 ->  [0.09,1,0]




# Chapter 3 Question 1
# [3.4,2.05]


