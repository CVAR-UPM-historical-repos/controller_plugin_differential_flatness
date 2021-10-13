/*!********************************************************************************
 * \brief     Differential Flatness controller Implementation
 * \authors   Miguel Fernandez-Cortizas
 * \copyright Copyright (c) 2020 Universidad Politecnica de Madrid
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 * 
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 * 3. Neither the name of the copyright holder nor the names of its contributors
 *    may be used to endorse or promote products derived from this software
 *    without specific prior written permission.
 * 
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE
 * OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *******************************************************************************/

#include "rclcpp/rclcpp.hpp"
#include "DF_controller.hpp"

//#include <dynamic_reconfigure/server.h>


int main(int argc, char * argv[])
{
  setvbuf(stdout, NULL, _IONBF, BUFSIZ);
  rclcpp::init(argc, argv);
  std::cout << "Starting Differential flatness controller " << std::endl;
  rclcpp::spin(std::make_shared<PD_controller>());
  rclcpp::shutdown();
  return 0;
}


/*
int main(int argc, char **argv)
{
    ros::init(argc, argv,"pd_controller_node");

    PD_controller my_controller;
    my_controller.setUp();


    #ifdef DYNAMIC_TUNING

        //dynamic Reconfigure
        dynamic_reconfigure::Server<pd_controller::ControllerConfig> server;
        dynamic_reconfigure::Server<pd_controller::ControllerConfig>::CallbackType f;

        f = boost::bind(&PD_controller::parametersCallback, &my_controller,_1, _2);
        server.setCallback(f);

    #endif

    ros::Rate rate(100);
    while(ros::ok())
    {
        //updating all the ros msgs
        ros::spinOnce();
        //running the localizer
        my_controller.run();
        rate.sleep();
    }

    return 0;
}
    */