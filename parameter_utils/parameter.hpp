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

		std::variant<int, std::string, bool, double, float> mDefaultValue;

		ParameterWrapper(std::string paramDescriptor, int& variable, int defaultValue = 0) : mType{rclcpp::ParameterType::PARAMETER_INTEGER}, mParamDescriptor{std::move(paramDescriptor)}, mData{&variable}, mDefaultValue{defaultValue}{}

		ParameterWrapper(std::string paramDescriptor, std::string& variable, std::string defaultValue = "") : mType{rclcpp::ParameterType::PARAMETER_STRING}, mParamDescriptor{std::move(paramDescriptor)}, mData{&variable}, mDefaultValue{std::move(defaultValue)}{}

		ParameterWrapper(std::string paramDescriptor, bool& variable, bool defaultValue = false) : mType{rclcpp::ParameterType::PARAMETER_BOOL}, mParamDescriptor{std::move(paramDescriptor)}, mData{&variable}, mDefaultValue{defaultValue}{}

		ParameterWrapper(std::string paramDescriptor, double& variable, double defaultValue = 0.0) : mType{rclcpp::ParameterType::PARAMETER_DOUBLE}, mParamDescriptor{std::move(paramDescriptor)}, mData{&variable}, mDefaultValue{defaultValue}{}

		ParameterWrapper(std::string paramDescriptor, float& variable, float defaultValue = 0.0) : mType{rclcpp::ParameterType::PARAMETER_DOUBLE}, mParamDescriptor{std::move(paramDescriptor)}, mData{&variable}, mDefaultValue{defaultValue}{}

		void visit(rclcpp::Node* node){
			std::visit(overload{
				[&](int* arg){
					try{
						*arg = static_cast<int>(node->get_parameter(mParamDescriptor).as_int());
					}catch(rclcpp::exceptions::ParameterUninitializedException& e){
						try{
							*arg = std::get<int>(mDefaultValue);
						}catch (std::bad_variant_access const& ex){
							throw std::runtime_error("Bad Variant Access: Type not int");
						}
					}
				},
				[&](std::string* arg){
					try{
							*arg = node->get_parameter(mParamDescriptor).as_string();
					}catch(rclcpp::exceptions::ParameterUninitializedException& e){
						try{
							*arg = std::get<std::string>(mDefaultValue);
						}catch (std::bad_variant_access const& ex){
							throw std::runtime_error("Bad Variant Access: Type not std::string");
						}
					}
				},
				[&](bool* arg){
					try{
						*arg = node->get_parameter(mParamDescriptor).as_bool();
					}catch(rclcpp::exceptions::ParameterUninitializedException& e){
						try{
							*arg = std::get<bool>(mDefaultValue);
						}catch (std::bad_variant_access const& ex){
							throw std::runtime_error("Bad Variant Access: Type not bool");
						}
					}
				},
				[&](double* arg){
					try{
						*arg = node->get_parameter(mParamDescriptor).as_double();
					}catch(rclcpp::exceptions::ParameterUninitializedException& e){
						try{
							*arg = std::get<double>(mDefaultValue);
						}catch (std::bad_variant_access const& ex){
							throw std::runtime_error("Bad Variant Access: Type not double");
						}
					}
				},
				[&](float* arg){
					try{
						*arg = static_cast<float>(node->get_parameter(mParamDescriptor).as_double());
					}catch(rclcpp::exceptions::ParameterUninitializedException& e){
						try{
							*arg = std::get<float>(mDefaultValue);
						}catch (std::bad_variant_access const& ex){
							throw std::runtime_error("Bad Variant Access: Type not float");
						}
					}
				}
			}, mData);
		}

		static inline auto declareParameters(rclcpp::Node* node, std::vector<ParameterWrapper>& params) -> void{
			RCLCPP_INFO(rclcpp::get_logger("param_logger"), "Declaring %zu parameters...", params.size());
			for(auto& param : params){
				node->declare_parameter(param.mParamDescriptor, param.mType);
				param.visit(node);
			}
		}
	};
};
