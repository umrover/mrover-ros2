#include <vector>
#include <string>
#include <variant>
#include <exception>
#include <rclcpp/rclcpp.hpp>

// Overloads pattern for visit
template<class... Ts> struct overload : Ts... { using Ts::operator()...; };
template<class... Ts> overload(Ts...) -> overload<Ts...>;


namespace mrover {
	class ParameterWrapper {
	private:
		static inline std::shared_ptr<rclcpp::ParameterCallbackHandle> cbHande;
		
	public:
		rclcpp::ParameterType mType;

		std::string mParamDescriptor;

		std::variant<int*, std::string*, bool*, double*, float*> mData;

		ParameterWrapper(std::string paramDescriptor, int& variable) : mType{rclcpp::ParameterType::PARAMETER_INTEGER}, mParamDescriptor{std::move(paramDescriptor)}, mData{&variable}{}

		ParameterWrapper(std::string paramDescriptor, std::string& variable) : mType{rclcpp::ParameterType::PARAMETER_STRING}, mParamDescriptor{std::move(paramDescriptor)}, mData{&variable}{}

		ParameterWrapper(std::string paramDescriptor, bool& variable) : mType{rclcpp::ParameterType::PARAMETER_BOOL}, mParamDescriptor{std::move(paramDescriptor)}, mData{&variable}{}

		ParameterWrapper(std::string paramDescriptor, double& variable) : mType{rclcpp::ParameterType::PARAMETER_DOUBLE}, mParamDescriptor{std::move(paramDescriptor)}, mData{&variable}{}

		ParameterWrapper(std::string paramDescriptor, float& variable) : mType{rclcpp::ParameterType::PARAMETER_DOUBLE}, mParamDescriptor{std::move(paramDescriptor)}, mData{&variable}{}

		void visit(rclcpp::Node* node, std::shared_ptr<rclcpp::ParameterEventHandler>& paramSub){
			std::visit(overload{
				[&](int* arg){
					try{
						*arg = static_cast<int>(node->get_parameter(mParamDescriptor).as_int());
					}catch(rclcpp::exceptions::ParameterUninitializedException& e){
						*arg = 0;
					}
				},
				[&](std::string* arg){
					try{
						*arg = node->get_parameter(mParamDescriptor).as_string();
					}catch(rclcpp::exceptions::ParameterUninitializedException& e){
						*arg = std::string();
					}
				},
				[&](bool* arg){
					try{
						*arg = node->get_parameter(mParamDescriptor).as_bool();
					}catch(rclcpp::exceptions::ParameterUninitializedException& e){
						*arg = false;
					}
				},
				[&](double* arg){
					try{
						*arg = node->get_parameter(mParamDescriptor).as_double();
					}catch(rclcpp::exceptions::ParameterUninitializedException& e){
						*arg = 0.0;
					}
				},
				[&](float* arg){
					try{
						*arg = static_cast<float>(node->get_parameter(mParamDescriptor).as_double());
					}catch(rclcpp::exceptions::ParameterUninitializedException& e){
						*arg = 0.0;
					}
				}
			}, mData);
		}

		static inline auto declareParameters(rclcpp::Node* node, std::shared_ptr<rclcpp::ParameterEventHandler>& paramSub, std::vector<ParameterWrapper>& params) -> void{
			RCLCPP_INFO(rclcpp::get_logger("param_logger"), "Declaring %zu parameters...", params.size());
			for(auto& param : params){
				node->declare_parameter(param.mParamDescriptor, param.mType);
				param.visit(node, paramSub);
			}
		}
	};
};
