#include "solver.h"

#include <boost/python.hpp>
#include <boost/python/numpy.hpp>
#include <boost/python/object.hpp>
#include <boost/python/manage_new_object.hpp>
#include <boost/python/return_value_policy.hpp>

namespace py = boost::python;
namespace np = boost::python::numpy;

using std::vector;
using std::cout;
using std::endl;

py::object NewAInstance() {
    
    auto result = std::make_shared<planning::Solver>();

    return py::object(result);
}

py::object BuildAOutput(const py::object &in_obj, py::object &data) {

    planning::SolverPtr instance_ptr = py::extract<planning::SolverPtr>(in_obj);
    vector<vector<vector<double>>> points;
    vector<vector<double>> agents, obstacles;
    vector<vector<int>> pre_connections, nxt_connections, left_connection, right_connection;
    vector<double> ego, lanes_length;
    vector<int> lanes_id;

    // data preprocessing
    // tuple(lanes_id, length_length points, connections, ego, vehicles, obstacles)
    py::tuple dataList = py::extract<py::tuple>(data);
    // for (int i = 0; i < )

    np::ndarray lanes_id_tmp = np::from_object((dataList.slice(0, 1))[0]);
    np::ndarray lanes_length_tmp = np::from_object((dataList.slice(1, 2))[0]);
    np::ndarray points_tmp = np::from_object((dataList.slice(2, 3))[0]);
    np::ndarray pre_connections_tmp = np::from_object((dataList.slice(3, 4))[0]);
    np::ndarray nxt_connections_tmp = np::from_object((dataList.slice(4, 5))[0]);
    np::ndarray left_connection_tmp = np::from_object((dataList.slice(5, 6))[0]);
    np::ndarray right_connection_tmp = np::from_object((dataList.slice(6, 7))[0]);
    np::ndarray ego_tmp = np::from_object(dataList.slice(7, 8));
    np::ndarray agents_tmp = np::from_object((dataList.slice(8, 9))[0]);
    np::ndarray obstacles_tmp = np::from_object((dataList.slice(9, 10))[0]);
        // one-dimension: 1 * num (shape(0))
        // row: shape(0), col: shape(1)

        // lanes_id(vector<int>, 1 * nums)
    // int* lanes_id_tmp_ptr = reinterpret_cast<int*>(lanes_id_tmp.get_data());
    for (int i = 0; i < static_cast<int>(lanes_id_tmp.shape(0)); ++i){
        int* lanes_id_tmp_ptr = reinterpret_cast<int*>(np::from_object(lanes_id_tmp[i]).get_data());
        lanes_id.push_back(*(lanes_id_tmp_ptr));
    }

        // lanes_length(vector<double>, 1 * nums)
    double* lanes_length_tmp_ptr = reinterpret_cast<double*>(lanes_length_tmp.get_data());
    for (int i = 0; i < static_cast<int>(lanes_length_tmp.shape(0)); i++){
        lanes_length.push_back(*(lanes_length_tmp_ptr + i));
    }
    

    // points(vector<vector<vector<double>>>, nums_lanes * nums_points * 2)
    for (int i = 0; i < points_tmp.shape(0); ++i) {
        vector<vector<double>> points_1;
        np::ndarray points_1_tmp = np::from_object(points_tmp[i]);
        for (int j = 0; j < points_tmp.shape(1); ++j) {
            vector<double> points_2;
            np::ndarray points_2_tmp = np::from_object(points_1_tmp[j]);
            double* points_tmp_ptr = reinterpret_cast<double*>(points_2_tmp.get_data());
            for (int k = 0; k < points_tmp.shape(2); ++k) {
                points_2.push_back(*(points_tmp_ptr + k));
            }
            points_1.push_back(points_2);
        }
        points.push_back(points_1);

    }   
        // pre_connections(vector<vector<int>>, nums_lanes * nums)
    for (int i = 0; i < pre_connections_tmp.shape(0); i++) {
        vector<int> pre_connections_2;
        np::ndarray pre_connections_2_tmp = np::from_object(pre_connections_tmp[i]);
        for (int j = 0; j < pre_connections_tmp.shape(1); j++) {
            int* pre_connections_tmp_ptr = reinterpret_cast<int*>(np::from_object(pre_connections_2_tmp[j]).get_data());
            pre_connections_2.push_back(*(pre_connections_tmp_ptr));
        }
        pre_connections.push_back(pre_connections_2);
    }

        // nxt_connections(vector<vector<int>>, nums_lanes * nums)
    for (int i = 0; i < nxt_connections_tmp.shape(0); i++) {
        vector<int> nxt_connections_2;
        np::ndarray nxt_connections_2_tmp = np::from_object(nxt_connections_tmp[i]);
        for (int j = 0; j < nxt_connections_tmp.shape(1); j++) {
            int* nxt_connections_tmp_ptr = reinterpret_cast<int*>(np::from_object(nxt_connections_2_tmp[j]).get_data());
            nxt_connections_2.push_back(*(nxt_connections_tmp_ptr));
        }
        nxt_connections.push_back(nxt_connections_2);
    }

        // left_connection(vector<vector<int>>, nums_lanes * (0 or 1))
    for (int i = 0; i < left_connection_tmp.shape(0); i++) {
        vector<int> left_connection_2;
        np::ndarray left_connection_2_tmp = np::from_object(left_connection_tmp[i]);
        for (int j = 0; j < left_connection_tmp.shape(1); j++) {
            int* left_connection_tmp_ptr = reinterpret_cast<int*>(np::from_object(left_connection_2_tmp[j]).get_data());
            left_connection_2.push_back(*(left_connection_tmp_ptr));
        }
        left_connection.push_back(left_connection_2);
    }

        // right_connection(vector<vector<int>>, nums_lanes * (0 or 1))
    for (int i = 0; i < right_connection_tmp.shape(0); i++) {
        vector<int> right_connection_2;
        np::ndarray right_connection_2_tmp = np::from_object(right_connection_tmp[i]);
        for (int j = 0; j < right_connection_tmp.shape(1); j++) {
            int* right_connection_tmp_ptr = reinterpret_cast<int*>(np::from_object(right_connection_2_tmp[j]).get_data());
            right_connection_2.push_back(*(right_connection_tmp_ptr));
        }
        right_connection.push_back(right_connection_2);
    }

        // ego(vector<double>, nums * 7)
    double* ego_tmp_ptr = reinterpret_cast<double*>(ego_tmp.get_data());
    for (int i = 0; i < static_cast<int>(ego_tmp.shape(1)); i++){
        ego.push_back(*(ego_tmp_ptr + i));
    }

        // agents(vector<vector<double>>, nums * 8)
        // pose((center)x, y, heading), velocity(x, y, heading), width, length
    for (int i = 0; i < agents_tmp.shape(0); i++) {
        vector<double> agents_2;
        np::ndarray agents_2_tmp = np::from_object(agents_tmp[i]);
        double* agents_tmp_ptr = reinterpret_cast<double*>(agents_2_tmp.get_data());
        for (int j = 0; j < agents_tmp.shape(1); j++) {
            agents_2.push_back(*(agents_tmp_ptr + j));
        }
        agents.push_back(agents_2);
    }

        // obstacles(vector<vector<double>>, nums * 4)
    for (int i = 0; i < obstacles_tmp.shape(0); i++) {
        vector<double> obstacles_2;
        np::ndarray obstacles_2_tmp = np::from_object(obstacles_tmp[i]);
        double* obstacles_tmp_ptr = reinterpret_cast<double*>(obstacles_2_tmp.get_data());
        for (int j = 0; j < obstacles_tmp.shape(1); j++) {
            obstacles_2.push_back(*(obstacles_tmp_ptr + j));
        }
        obstacles.push_back(obstacles_2);
    }


    // data -> solver 
    auto result = instance_ptr->solver(lanes_id, lanes_length, points, pre_connections, 
            nxt_connections, left_connection, right_connection, ego, agents, obstacles);

    py::list ret_lst;
    for (int i = 0; i < static_cast<int>(result.size()); i++){
        py::list ret_lst_tmp;
        for (int j = 0; j < static_cast<int>(result[i].size()); j++) {
            ret_lst_tmp.append(result[i][j]);
        }
        ret_lst.append(ret_lst_tmp);
    }

    return ret_lst;
}

// 定义python绑定C++函数名称
BOOST_PYTHON_MODULE(libpy_solver_util) {

Py_Initialize();
np::initialize();

py::class_<planning::SolverPtr>("NewAClassIdentity", py::init<>());

py::def("new_a_instance", &NewAInstance);

py::def("build_a_output", &BuildAOutput);
}