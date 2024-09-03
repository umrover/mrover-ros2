#include <vector>
#include <string>
#include <variant>
#include <exception>
#include <format>
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

		std::variant<int*, std::string*, bool*> mData;

		ParameterWrapper(std::string paramDescriptor, int& variable) : mType{rclcpp::ParameterType::PARAMETER_INTEGER}, mParamDescriptor{std::move(paramDescriptor)}, mData{&variable}{}

		ParameterWrapper(std::string paramDescriptor, std::string& variable) : mType{rclcpp::ParameterType::PARAMETER_STRING}, mParamDescriptor{std::move(paramDescriptor)}, mData{&variable}{}

		ParameterWrapper(std::string paramDescriptor, bool& variable) : mType{rclcpp::ParameterType::PARAMETER_BOOL}, mParamDescriptor{std::move(paramDescriptor)}, mData{&variable}{}

		void visit(rclcpp::Node* node, std::shared_ptr<rclcpp::ParameterEventHandler>& paramSub){
			std::visit(overload{
				[&](int* arg){
				cbHande = paramSub->add_parameter_callback(mParamDescriptor, [&](rclcpp::Parameter const& rclcppParam) {
						try{
							*arg = static_cast<int>(rclcppParam.as_int());
							RCLCPP_INFO(node->get_logger(), "Recieved %s as %d", mParamDescriptor.c_str(), *arg);
						}catch(const std::bad_variant_access& e){
							throw std::runtime_error(std::format("Bad variant access while recieving parameter"));
						}
					});
				},
				[&](std::string* arg){
					ParameterWrapper::cbHande = paramSub->add_parameter_callback(mParamDescriptor, [&](rclcpp::Parameter const& rclcppParam) {
						try{
							*arg = rclcppParam.as_string();
						}catch(const std::bad_variant_access& e){
							throw std::runtime_error(std::format("Bad variant access while recieving parameter"));
						}
					});
				},
				[&](bool* arg){
					ParameterWrapper::cbHande = paramSub->add_parameter_callback(mParamDescriptor, [&](rclcpp::Parameter const& rclcppParam) {
						try{
							*arg = rclcppParam.as_bool();
						}catch(const std::bad_variant_access& e){
							throw std::runtime_error(std::format("Bad variant access while recieving parameter"));
						}
					});
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
