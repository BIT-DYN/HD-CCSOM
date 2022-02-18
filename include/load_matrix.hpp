#ifndef LOAD_MATRIX_HPP
#define LOAD_MATRIX_HPP

#include <ros/ros.h>
#include <iostream>
#include <fstream>
#include <string>
#include <XmlRpcException.h>
#include <XmlRpcValue.h>
bool load_matrix(ros::NodeHandle &nh_, std::string param_name, Eigen::Matrix4f &Matrix)
{
    XmlRpc::XmlRpcValue matrix_xml;

    try
    {
        if (nh_.getParam(param_name, matrix_xml))
        {
            ROS_ASSERT(matrix_xml.getType() == XmlRpc::XmlRpcValue::TypeArray);
            int matSize = Matrix.rows();

            for (int i = 0; i < matSize; i++)
            {
                for (int j = 0; j < matSize; j++)
                {
                    try
                    {
                        std::ostringstream ostr;
                        ostr << matrix_xml[matSize * i + j];
                        std::istringstream istr(ostr.str());
                        istr >> Matrix(i, j);
                    }
                    catch (XmlRpc::XmlRpcException &e)
                    {
                        throw e;
                    }
                    catch (...)
                    {
                        throw;
                    }
                }
            }
        }
        else
        {
            Matrix << 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1;
            std::cout << param_name << " "
                      << "Use identity matrix" << std::endl;
        }
    }
    catch (XmlRpc::XmlRpcException &e)
    {
        ROS_ERROR_STREAM("ERROR reading matrix param config: " << e.getMessage()
                                                               << " for Matrix (type: " << matrix_xml.getType() << ")");
    }
    std::cout << " Matrix: " << std::endl
              << Matrix << std::endl;
    return true;
}
#endif