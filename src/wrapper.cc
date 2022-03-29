#include "solver.h"

#include <boost/python.hpp>
#include <boost/python/numpy.hpp>
#include <boost/python/object.hpp>
#include <boost/python/manage_new_object.hpp>
#include <boost/python/return_value_policy.hpp>

boost::python::object NewAInstance() {
    boost::python::numpy::initialize();
    auto result = std::make_shared<planning::Solver>();

    return boost::python::object(result);
}

boost::python::object BuildAOutput(const boost::python::object &in_obj) {
    planning::SolverPtr instance_ptr = boost::python::extract<planning::SolverPtr>(in_obj);

    auto result = instance_ptr->solver(0, 
    "/home/jp/ego_solver/config/highway_v1.0/agent_config.json",
    "/home/jp/ego_solver/core/eudm_planner/config/eudm_config.pb.txt",
    "/home/jp/ego_solver/core/ssc_planner/config/ssc_config.pb.txt",
    "/home/jp/ego_solver/config/highway_v1.0/vehicle_set.json",
    "/home/jp/ego_solver/config/highway_v1.0/obstacles_norm.json",
    "/home/jp/ego_solver/config/highway_v1.0/lane_net_norm.json", 20);

    boost::python::list ret_lst;

    for (int i = 0; i < result.size(); i++){
        boost::python::list ret_lst_tmp;
        for (int j = 0; j < result[i].size(); j++) {
            ret_lst_tmp.append(result[i][j]);
        }
        ret_lst.append(ret_lst_tmp);
    }


    return ret_lst;
}

// 定义python绑定C++函数名称
BOOST_PYTHON_MODULE(libpy_solver_util) {

boost::python::class_<planning::SolverPtr>("NewAClassIdentity", boost::python::init<>());

boost::python::def("new_a_instance", &NewAInstance);

boost::python::def("build_a_output", &BuildAOutput);
}