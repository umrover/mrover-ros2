#pragma once

#include <initializer_list>
#include <string>
#include <exception>
#include <rclcpp/rclcpp.hpp>

namespace mrover {
	class ParameterWrapper {
	public:
		ParameterWrapper(rclcpp::Node* node, std::string const& paramDescriptor, auto& variable, std::decay_t<typename std::remove_reference<decltype(variable)>::type> defaultValue = std::remove_reference<decltype(variable)>::type()){
			using T = std::remove_reference<decltype(variable)>::type;
			
			try{
				if constexpr (std::is_same_v<T, bool>){
					node->declare_parameter(paramDescriptor, rclcpp::ParameterType::PARAMETER_BOOL);
					variable = node->get_parameter(paramDescriptor).as_bool(); 
				}else if constexpr (std::is_integral_v<T>){
					node->declare_parameter(paramDescriptor, rclcpp::ParameterType::PARAMETER_INTEGER);
					variable = node->get_parameter(paramDescriptor).as_int(); 
				}else if constexpr (std::is_floating_point_v<T>){
					node->declare_parameter(paramDescriptor, rclcpp::ParameterType::PARAMETER_DOUBLE);
					variable = node->get_parameter(paramDescriptor).as_double(); 
				}else if constexpr (std::is_convertible_v<T, std::string_view>){
					node->declare_parameter(paramDescriptor, rclcpp::ParameterType::PARAMETER_STRING);
					variable = node->get_parameter(paramDescriptor).as_string();
				}else{
					throw std::runtime_error("Invalid Parameter Type");
				}
			}catch(rclcpp::exceptions::ParameterUninitializedException& e){
				variable = defaultValue;
			}		
		}
	};
};