import libpy_solver_util as py_solver_util
import numpy as np


def main():
	in_obj = py_solver_util.new_a_instance()

	# lanes_1 = np.array([[1, 2], [3, 4], [5, 6], [7, 8]]).tolist()
	# lanes_2 = np.array([[4, 5], [6, 7]]).tolist()
	# ego = np.array([1.0, 2.0, 3.0, 4.0, 5.0]).tolist()
	# agents = np.zeros([2,3]).tolist()
	# data = list((lanes_1, lanes_2, ego, agents))
	
	lanes_id = np.array([54, 55, 56, 57])
	lanes_length = np.array([55.1, 56.2, 58.3, 57.1])

	points = np.array([[[1.0, 2.0], [3.0, 4.0], [5.0, 6.0], [7.0, 8.0]],
			[[9.0, 10.0], [11.0, 12.0], [13.0, 14.0], [15.0, 16.0]], 
			[[17.0, 18.0], [19.0, 20.0], [21.0, 22.0], [23.0, 24.0]]])

	pre_connections = np.array([[1654, 1458, 1235], [1654, 1458, 1235], [1654, 1458, 1235]])
	nxt_connections = np.array([[1654, 1458, 1235], [1654, 1458, 1235], [1654, 1458, 1235]])
	left_connection = np.array([[1654, 1458, 1235], [1654, 1458, 1235], [1654, 1458, 1235]])
	right_connection = np.array([[1654, 1458, 1235], [1654, 1458, 1235], [1654, 1458, 1235]])

	ego = np.array([1.0, 2.0, 3.0, 4.0, 5.0])
	agents = np.array([[2.89901009e+01, -1.23965216e+01, -2.48171352e-02,
         3.84007543e-01, -7.53674567e-01,  1.23427089e-15,
         6.88737214e-01,  6.45051718e-01], 
		 [8.60684490e+00,  3.05599213e+00, -4.37319605e-03,
         3.84007543e-01, -7.53674567e-01, -1.75208257e-15,
         2.16140175e+00,  5.99200773e+00], 
		 [3.88407936e+01, -1.21819992e+01, -4.02741656e-02,
         3.84007543e-01, -7.53674567e-01,  2.43377045e-15,
         7.45388806e-01,  8.46675992e-01]])
	obstacles = np.array([[1.2, 2.3, 3.4, 4.5], [1.2, 2.3, 3.4, 4.5],[1.2, 2.3, 3.4, 4.5]])
	data = tuple((lanes_id, lanes_length, points, pre_connections, nxt_connections, 
		left_connection, right_connection, ego, agents, obstacles))
	
	print(len(data))
	result = py_solver_util.build_a_output(in_obj, data)
	print("You succeed calling C++ using Python, here is your result (x, y, velocity, acceleration, curvature:  ):")
	for i in range(len(result)):
		print(result[i])


if __name__ == '__main__':
	main()
