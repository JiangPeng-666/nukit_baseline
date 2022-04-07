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
    vector<vector<double>> lanes, agents;
    vector<vector<int>> connections;
    vector<double> ego;

    // data preprocessing
    py::tuple dataList = py::extract<py::tuple>(data);
    np::ndarray lanes_tmp = np::from_object((dataList.slice(0, 1))[0]);
    np::ndarray connections_tmp = np::from_object((dataList.slice(1, 2))[0]);
    np::ndarray ego_tmp = np::from_object(dataList.slice(2, 3));
    np::ndarray agents_tmp = np::from_object((dataList.slice(3, 4))[0]);
        // row: shape(0), col: shape(1)

        // lanes(vector<vector<double>>, nums * 2)
    for (int i = 0; i < lanes_tmp.shape(0); i++) {
        vector<double> lanes_2;
        np::ndarray lanes_2_tmp = np::from_object(lanes_tmp[i]);
        double* lanes_tmp_ptr = reinterpret_cast<double*>(lanes_2_tmp.get_data());
        for (int j = 0; j < lanes_tmp.shape(1); j++) {
            lanes_2.push_back(*(lanes_tmp_ptr + j));
        }
        lanes.push_back(lanes_2);
    }

        // connections(vector<vector<int>>, nums * 2)
    for (int i = 0; i < connections_tmp.shape(0); i++) {
        vector<int> connections_2;
        np::ndarray connections_2_tmp = np::from_object(connections_tmp[i]);
        for (int j = 0; j < connections_tmp.shape(1); j++) {
            int* connections_tmp_ptr = reinterpret_cast<int*>(np::from_object(connections_2_tmp[j]).get_data());
            connections_2.push_back(*(connections_tmp_ptr));
        }
        connections.push_back(connections_2);
    }

        // ego(vector<double>, nums * 5)
    double* ego_tmp_ptr = reinterpret_cast<double*>(ego_tmp.get_data());
    for (int i = 0; i < static_cast<int>(ego_tmp.shape(1)); i++){
        ego.push_back(*(ego_tmp_ptr + i));
    }

        // agents(vector<vector<double>>, nums * 8)
    for (int i = 0; i < agents_tmp.shape(0); i++) {
        vector<double> agents_2;
        np::ndarray agents_2_tmp = np::from_object(agents_tmp[i]);
        double* agents_tmp_ptr = reinterpret_cast<double*>(agents_2_tmp.get_data());
        for (int j = 0; j < agents_tmp.shape(1); j++) {
            agents_2.push_back(*(agents_tmp_ptr + j));
        }
        agents.push_back(agents_2);
    }

    // data -> solver 
    auto result = instance_ptr->solver(lanes, connections, ego, agents);

    // result
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