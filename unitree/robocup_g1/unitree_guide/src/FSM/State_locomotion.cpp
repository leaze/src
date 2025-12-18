/**********************************************************************
 Copyright (c) 2020-2025, Unitree Robotics.Co.Ltd. All rights reserved.
***********************************************************************/
#include <iostream>
#include "FSM/State_locomotion.h"
#include <onnxruntime_cxx_api.h>
#include <cmath>
#include <numeric>  // 添加这个头文件用于std::accumulate
#include <functional> // 用于std::multiplies
#include <utility>
#include <fstream>

State_locomotion::State_locomotion(CtrlComponents *ctrlComp)
    : FSMState(ctrlComp, FSMStateName::LOCOMOTION, "LOCOMOTION")
{
    this->needcheckSafty = false;
    int ret1 = pthread_mutex_init(&write_cmd_mutex, NULL);
    loadPolicy();
}

void State_locomotion::loadPolicy()
{
    std::string policy_file_path;
#ifdef COMPILE_WITH_CATKIN_MAKE
#ifdef ROBOT_TYPE_G1
    std::string env_cfg_path = getWorkingDir() + "/src/unitree_guide/data/policy/motion/a1.yaml";
    env_cfg = YAML::LoadFile(env_cfg_path.c_str());
    yamlParam_.setup(env_cfg_path.c_str());
    // policy_file_path = getWorkingDir() + "/src/unitree_guide/data/policy/motion/x1_policy_conservative.mnn";
    // policy_file_path = getWorkingDir() + "/src/unitree_guide/data/policy/motion/actor_1.onnx";
    policy_file_path = yamlParam_.ReadStringFromYaml("policy_file_path");
    policy_file_path = getWorkingDir() + policy_file_path;
#endif
#else
#endif

    std::cout << "[State_locomotion] policy_file_path: " << policy_file_path << std::endl;

    // 改为ONNX代码
    env = std::make_unique<Ort::Env>(ORT_LOGGING_LEVEL_WARNING, "UnitreeRobot");
    // 2. 配置会话选项
    session_options.SetIntraOpNumThreads(2);
    session_options.SetGraphOptimizationLevel(GraphOptimizationLevel::ORT_ENABLE_ALL);

    // 3. 加载模型,创建推理会话
    session = std::make_unique<Ort::Session>(*env, policy_file_path.c_str(), session_options);

    // 4. 获取输入输出信息
    Ort::AllocatorWithDefaultOptions allocator;
    num_observations = 47;
    num_actions = 12;

    // Configure frame stacking: 66 frames * 47 features = 3102 input size
    frame_stack_size = 66; // stack 66 frames
    stacked_input_size = num_observations * frame_stack_size; // 47 * 66 = 3102
    // stacked_input_size = num_observations;

    input_tensor_values.resize(stacked_input_size);
    input_shape = {1, stacked_input_size}; // batch_size=1, features=3102

    // initialize stacked observations with zeros
    stacked_obs.assign(stacked_input_size, 0.0f);

    // check environment variable to enable dumping policy inputs
    // const char* dump_env = std::getenv("DUMP_POLICY_INPUT");
    std::string strDUMP_POLICY_INPUT = yamlParam_.ReadStringFromYaml("LOG_switch");
    const char* dump_env = strDUMP_POLICY_INPUT.c_str();
    CreateDumpFile(dump_env);    

    // 5. 获取模型输入信息
    std::cout << "Tensors Info:" << std::endl;
    std::cout << "---------------------------------" << std::endl;

    // 获取输入节点数量
    size_t num_input_nodes = session->GetInputCount();
    for (size_t i = 0; i < num_input_nodes; i++) {
        // 获取输入节点信息
        // 获取输入名称
        auto input_name_ptr = session->GetInputNameAllocated(i, allocator);
        input_name = input_name_ptr.get(); // 转换为std::string
        std::cout << "Tensor Name: " << input_name << std::endl;
        
        // 获取输入形状信息
        auto type_info = session->GetInputTypeInfo(i);
        auto tensor_info = type_info.GetTensorTypeAndShapeInfo();
        auto shape = tensor_info.GetShape();

        // 打印形状信息
        std::cout << "  Shape: [";
        for (size_t j = 0; j < shape.size(); j++) {
            std::cout << shape[j];
            if (j < shape.size() - 1) std::cout << ", ";
        }
        std::cout << "]" << std::endl;
        
        // 打印元素数量
        if (std::find(shape.begin(), shape.end(), -1) != shape.end()) {
            std::cout << "  Element Size: dynamic (contains -1)" << std::endl;
        } else {
            size_t element_count = std::accumulate(shape.begin(), shape.end(), 1, std::multiplies<int64_t>());
            std::cout << "  Element Size: " << element_count << std::endl;
        }
        std::cout << "---------------------------------" << std::endl;

    }
    size_t num_output_nodes = session->GetOutputCount();
    for (size_t i = 0; i < num_input_nodes; i++) {
        // 获取输出节点信息
        // 获取输出名称
        auto output_name_ptr = session->GetOutputNameAllocated(i, allocator);
        output_name = output_name_ptr.get(); // 转换为std::string
        std::cout << "Tensor Name: " << output_name << std::endl;
        
        // 获取输出形状信息
        auto type_info = session->GetOutputTypeInfo(i);
        auto tensor_info = type_info.GetTensorTypeAndShapeInfo();
        auto shape = tensor_info.GetShape();

        // 打印形状信息
        std::cout << "  Shape: [";
        for (size_t j = 0; j < shape.size(); j++) {
            std::cout << shape[j];
            if (j < shape.size() - 1) std::cout << ", ";
        }
        std::cout << "]" << std::endl;
        
        // 打印元素数量
        if (std::find(shape.begin(), shape.end(), -1) != shape.end()) {
            std::cout << "  Element Size: dynamic (contains -1)" << std::endl;
        } else {
            size_t element_count = std::accumulate(shape.begin(), shape.end(), 1, std::multiplies<int64_t>());
            std::cout << "  Element Size: " << element_count << std::endl;
        }
        std::cout << "---------------------------------" << std::endl;

    }
    // obs_tensor = net->getSessionInput(session, "input");
    // act_tensor = net->getSessionOutput(session, "output");

    // 在前面已经获取了输入输出名称
    auto input_name_ptr = session->GetInputNameAllocated(0, allocator);
    auto output_name_ptr = session->GetOutputNameAllocated(0, allocator);
    input_name = input_name_ptr.get();
    output_name = output_name_ptr.get();

    // 获取输入输出的形状信息
    auto input_type_info = session->GetInputTypeInfo(0);
    auto input_tensor_info = input_type_info.GetTensorTypeAndShapeInfo();
    input_shape = input_tensor_info.GetShape();

    auto output_type_info = session->GetOutputTypeInfo(0);
    auto output_tensor_info = output_type_info.GetTensorTypeAndShapeInfo();
    output_shape = output_tensor_info.GetShape();

    // 创建输入输出张量的内存
    auto memory_info = Ort::MemoryInfo::CreateCpu(OrtArenaAllocator, OrtMemTypeDefault);

    // 为输入分配内存 (frame-stacked: 66 * 47 = 3102)
    input_tensor_values.resize(stacked_input_size);
    input_shape = {1, stacked_input_size};  // 设置为[1, 3102]的形状

    // 为输出分配内存 (12维的动作向量)
    output_tensor_values.resize(num_actions);
    output_shape = {1, num_actions};  // 设置为[1, 12]的形状

    // 创建输入张量
    input_tensor = Ort::Value::CreateTensor<float>(
        memory_info, 
        input_tensor_values.data(), 
        input_tensor_values.size(),
        input_shape.data(), 
        input_shape.size()
    );

    // 验证张量维度
    std::cout << "Input tensor shape: [";
    for (const auto& dim : input_shape) {
        std::cout << dim << ", ";
    }
    std::cout << "]" << std::endl;

    dt = yamlParam_.ReadFloatFromYaml("control_dt");
    stiffness = yamlParam_.ReadVectorFromYaml("kps", num_actions);
    damping = yamlParam_.ReadVectorFromYaml("kds", num_actions);

    obs_scaled.setZero(num_observations);
    obs_current.setZero(num_observations);
    act_scaled.setZero(num_actions);
    act_temp.setZero(num_actions);

    obs_mean.setZero(num_observations);
    // default joint angles should align with joint position entries in obs_current,
    // which start at index 5 (obs_current(5 + j)). Use segment<12>(5) accordingly.
    obs_mean.segment<12>(5) = yamlParam_.ReadVectorFromYaml("default_angles", num_actions);

    obsScaleAngularVelocity = yamlParam_.ReadFloatFromYaml("ang_vel_scale"); // 角速度命令缩放
    float obsScaleDofPosition = yamlParam_.ReadFloatFromYaml("dof_pos_scale");
    float obsScaleDofVelocity = yamlParam_.ReadFloatFromYaml("dof_vel_scale");
    float obsScaleLastActions = yamlParam_.ReadFloatFromYaml("action_scale");
    obsScaleLinearVelocity = yamlParam_.ReadFloatFromYaml("lin_vel_scale");    // 线速度命令缩放

    obs_scales.setZero(num_observations);
    obs_scales.segment<2>(0) = Eigen::Vector2f::Ones();                                  // Phase (sin,cos) 2
    // 线速度和角速度命令使用各自的缩放系数
    obs_scales.segment<2>(2) = Eigen::Vector2f::Ones() * obsScaleLinearVelocity;        // Linear velocity commands (x,y) 2
    obs_scales(4) = obsScaleAngularVelocity;                                             // Angular velocity command (yaw) 1
    obs_scales.segment<12>(5) = Eigen::Vector<float, 12>::Ones() * obsScaleDofPosition;  // Joint position 12
    obs_scales.segment<12>(17) = Eigen::Vector<float, 12>::Ones() * obsScaleDofVelocity; // Joint velocity 12
    obs_scales.segment<12>(29) = Eigen::Vector<float, 12>::Ones();                       // Last actions 12
    obs_scales.segment<3>(41) = Eigen::Vector3f::Ones() * obsScaleAngularVelocity;      // Angular velocity (gyro) 3
    obs_scales.segment<3>(44) = Eigen::Vector3f::Ones();                                 // Euler angles 3

    act_mean.setZero(num_actions);
    act_mean.segment<12>(0) = yamlParam_.ReadVectorFromYaml("default_angles", num_actions);

    act_scales.setZero(num_actions);
    act_scales.segment<12>(0) = Eigen::Vector<float, 12>::Ones() * obsScaleLastActions;

    std::cout << "]: " << std::endl;

    act_prev.setZero(num_actions);
    obs_current.setZero(num_actions);

    if (dump_policy_input && input_file.is_open()) {
        // write a line: counter,timestamp,vals...
        long long ts = getSystemTime();
        input_file << 0 << "," << ts;
        for (int i = 0; i < num_observations; ++i) 
            input_file << "," << obs_scales(i);
        input_file << "\n";
        input_file.flush();
    }

    std::cout << "---------------- debug ----------------  " << std::endl;
    std::cout << "[State_locomotion::loadPolicy] dt: \n"
              << dt << std::endl;
    std::cout << "[State_locomotion::loadPolicy] stiffness: \n"
              << stiffness << std::endl;
    std::cout << "[State_locomotion::loadPolicy] damping: \n"
              << damping << std::endl;
    std::cout << "[State_locomotion::loadPolicy] obsScaleAngularVelocity: \n"
              << obsScaleAngularVelocity << std::endl;
    std::cout << "[State_locomotion::loadPolicy] obsScaleDofPosition: \n"
              << obsScaleDofPosition << std::endl;
    std::cout << "[State_locomotion::loadPolicy] obsScaleDofVelocity: \n"
              << obsScaleDofVelocity << std::endl;
    std::cout << "[State_locomotion::loadPolicy] obsScaleLastActions: \n"
              << obsScaleLastActions << std::endl;
}

void State_locomotion::enter()
{
    std::cout << "[State_locomotion] enter " << std::endl;
    counter = 0;

    // === 设置初始速度命令（与 sim2sim-onnx.py 一致） ===
    /*_lowState->userValue.ly = 0.1f;   // 向前速度（x方向）
    _lowState->userValue.lx = 0.0f;   // 横向速度（y方向）
    _lowState->userValue.rx = 0.0f;   // 偏航角速度*/

    _lowState->userValue.ly = 0.13f;   // 向前速度（x方向）
    _lowState->userValue.lx = 0.0f;   // 横向速度（y方向）
    _lowState->userValue.rx = 0.12f;   // 偏航角速度
    
    threadRunning = true;
    _thread = std::thread(&State_locomotion::inference, this);
}

void State_locomotion::inference()
{
    std::cout << "[inference] enter " << std::endl;
    while (threadRunning)
    {
        _startTime = getSystemTime();

        counter += 1;
        std::cout << "counter=" << counter<< std::endl;
        if ((int)counter % 5 == 0)
        {
            obs_current.setZero(num_observations);
            // === 1. phase (sin, cos) ===
            float period = 0.7;
            float count = counter * dt;
            float phase = std::fmod(static_cast<float>(count), period) / period;
            float sin_phase = std::sin(2 * M_PI * phase);
            float cos_phase = std::cos(2 * M_PI * phase);
            obs_current(0) = sin_phase;
            obs_current(1) = cos_phase;
    
            // === 2. command (x_vel, y_vel, yaw_vel) ===
            _userValue = _lowState->userValue;
            // 注意sim2sim顺序: x_vel, y_vel, yaw_vel
            
            /*
            obs_current(2) = _userValue.ly;      // x_vel_cmd
            obs_current(3) = -_userValue.lx;     // y_vel_cmd
            obs_current(4) = -_userValue.rx;     // yaw_vel_cmd
            */
            /*
            obs_current(2) = 0.13;      // x_vel_cmd
            obs_current(3) = 0.0;    // y_vel_cmd
            obs_current(4) = 0.0;     // yaw_vel_cmd*/
            obs_current(2) = _userValue.ly;      // x_vel_cmd
            obs_current(3) = -_userValue.lx;     // y_vel_cmd
            obs_current(4) = _userValue.rx;     // yaw_vel_cmd
            std::cout << "_userValue.ly=" << obs_current(2) << ", _userValue.lx=" << obs_current(3)<< ", _userValue.rx="<< obs_current(4) << std::endl;

            // std::cout << "q, dq,";
            for (int j = 0; j < 12; ++j)
            {
                // === 3. joint pos (q - default) * scale ===
                obs_current(5 + j) = _lowState->motorState[j].q;
    
                // === 4. joint vel (dq) * scale ===
                obs_current(17 + j) = _lowState->motorState[j].dq;
            }
    
            // === 5. last action ===
            obs_current.segment<12>(29) << act_prev;
    
            // === 6. omega (gyro) ===
            Eigen::Vector3d gyro(_lowState->getGyro()[0], _lowState->getGyro()[1], _lowState->getGyro()[2]);
            obs_current(41) = gyro[0] ;
            obs_current(42) = gyro[1] ;
            obs_current(43) = gyro[2] ;
            // Debug: print gyro values
            // std::cout << "Gyro readings - x:" << gyro[0] << " y:" << gyro[1] << " z:" << gyro[2] << std::endl;
    
            // === 7. euler angles ===
            // 由旋转矩阵转欧拉角（ZYX顺序）
            _B2G_RotMat = _lowState->getRotMat();
            double roll = atan2(_B2G_RotMat(2,1), _B2G_RotMat(2,2));
            double pitch = asin(-_B2G_RotMat(2,0));
            double yaw = atan2(_B2G_RotMat(1,0), _B2G_RotMat(0,0));
    
            // 将欧拉角归一化到[-π, π]范围
            auto normalize_angle = [](double angle) {
                while (angle > M_PI) angle -= 2 * M_PI;
                while (angle < -M_PI) angle += 2 * M_PI;
                return angle;
            };
            roll = normalize_angle(roll);
            pitch = normalize_angle(pitch);
            yaw = normalize_angle(yaw);
    
            obs_current(44) = roll;
            obs_current(45) = pitch;
            obs_current(46) = yaw;
    
            // 更新缩放后的观测值（确保后续的帧堆叠使用最新的缩放观测）
            obs_scaled = (obs_current - obs_mean).cwiseProduct(obs_scales);
    
            // 1. 准备输入数据 - 使用rolling buffer来堆叠帧
            // 将新的obs_scaled推入stacked_obs（左移旧帧，末尾追加新帧）
            // stacked_obs的布局: [frame0(47), frame1(47), ..., frame65(47)]
            // 每次将数据左移一帧大小，然后把最新的obs_scaled复制到末尾
            int frame_bytes = num_observations;
            // 左移 (丢弃最旧的一帧)
            if (stacked_obs.size() >= (size_t)stacked_input_size) {
                // move data from index frame_bytes .. end -> 0 .. end-frame_bytes
                std::move(stacked_obs.begin() + frame_bytes, stacked_obs.end(), stacked_obs.begin());
                // copy new frame to the tail
                for (int i = 0; i < num_observations; ++i) {
                    float v = obs_scaled(i);
                    if (std::isnan(v) || std::isinf(v) || std::abs(v) > 100.0f) {
                        std::cerr << "❌ 检测到无效输入值 at index " << i << ": " << v << std::endl;
                        // on invalid input, zero the new frame
                        stacked_obs[stacked_obs.size() - frame_bytes + i] = 0.0f;
                    } else {
                        stacked_obs[stacked_obs.size() - frame_bytes + i] = v;
                    }
                }
            } else {
                // 初始填充阶段，直接把新帧追加
                for (int i = 0; i < num_observations; ++i) {
                    float v = obs_scaled(i);
                    stacked_obs.push_back(std::isfinite(v) ? v : 0.0f);
                }
                // 如果还没有填满，前面会保留0值
                if (stacked_obs.size() < (size_t)stacked_input_size) {
                    stacked_obs.resize(stacked_input_size, 0.0f);
                }
            }
    
            // 将stacked_obs拷贝到input_tensor_values（flattened）
            std::copy(stacked_obs.begin(), stacked_obs.end(), input_tensor_values.begin());
            // 定期打印前几个输入值用于调试
            if ((int)counter % 100 == 0 ) {           
                if (dump_policy_input && input_file.is_open()) {
                    // write a line: counter,timestamp,vals...
                    long long ts = getSystemTime();
                    input_file << counter << "," << ts;
                    for (int i = 0; i < stacked_input_size; ++i) input_file << "," << input_tensor_values[i];
                    input_file << "\n";
                    input_file.flush();
                }
            }
    
            // 改为ONNX推理代码
            // 2. 创建内存信息
            auto memory_info = Ort::MemoryInfo::CreateCpu(OrtAllocatorType::OrtArenaAllocator, OrtMemType::OrtMemTypeDefault);
            // 3. 创建输入张量
            input_tensor = Ort::Value::CreateTensor<float>(
                memory_info, input_tensor_values.data(), input_tensor_values.size(),
                input_shape.data(), input_shape.size());
    
            // 4. 运行推理
            // 准备输入输出名称数组
            const char* input_names[] = {input_name.c_str()};
            const char* output_names[] = {output_name.c_str()};
    
            auto output_tensors = session->Run(
                Ort::RunOptions{nullptr},
                input_names, &input_tensor, 1,
                output_names, 1
            );
    
            // 5. 处理输出 - 将结果拷贝回Eigen向量
            float* output_data = output_tensors[0].GetTensorMutableData<float>();
            size_t output_size = output_tensors[0].GetTensorTypeAndShapeInfo().GetElementCount();
            
            // 确保输出维度匹配
            if (output_size == act_prev.size()) 
            {
                // 检查并拷贝输出数据到 act_prev（带有效性检查）
                bool has_invalid = false;
                for (int i = 0; i < (int)act_prev.size(); ++i) {
                    float val = output_data[i];
                    if (!std::isfinite(val) || std::abs(val) > 10.0f) {
                        has_invalid = true;
                        break;
                    }
                    act_prev(i) = val;
                }
    
                if (has_invalid) {
                    std::cerr << "❌ 检测到无效输出值，使用安全值代替" << std::endl;
                    act_prev = act_mean;  // 使用安全的默认姿态
                } 
            } 
            else 
            {
                std::cerr << "❌ 输出维度不匹配: 期望 " << act_prev.size() << ", 实际 " << output_size << std::endl;
                act_prev = act_mean;  // 使用安全的默认姿态
            }
    
            if ((int) counter % 100 == 0)
            {
                if (dump_policy_output && output_file.is_open()) {
                    // write a line: counter,timestamp,vals...
                    long long ts = getSystemTime();
                    output_file << counter << "," << ts;
                    for (int i = 0; i < output_size; ++i) output_file << "," << act_prev(i);
                    output_file << "\n";
                    output_file.flush();
                }
            }
        }

        pthread_mutex_lock(&write_cmd_mutex);
        act_scaled = act_prev.cwiseProduct(act_scales);// + act_mean;
        pthread_mutex_unlock(&write_cmd_mutex);
        absoluteWait(_startTime, (long long)(dt * 1000000));
    }
    std::cout << "end" << std::endl;
    threadRunning = false;
    std::cout << "done!" << std::endl;
}

void State_locomotion::run()
{
    pthread_mutex_lock(&write_cmd_mutex);
    memcpy(act_temp.data(), act_scaled.data(), act_scaled.size() * sizeof(float));
    pthread_mutex_unlock(&write_cmd_mutex);

    // torque_out = (target_q + cfg.robot_config.default_dof_pos - q ) * kp - dq * kd
    for (int i = 0; i < 12; i++)
    {
        // _lowCmd->motorCmd[i].q = act_temp[i];
        // _lowCmd->motorCmd[i].Kp = stiffness(i);
        // _lowCmd->motorCmd[i].Kd = damping(i);
        // _lowCmd->motorCmd[i].dq = 0;
        // _lowCmd->motorCmd[i].tau = 0;
        
        _lowCmd->motorCmd[i].q = act_temp[i];
        _lowCmd->motorCmd[i].Kp = stiffness(i);
        _lowCmd->motorCmd[i].Kd = damping(i);
        _lowCmd->motorCmd[i].dq = 0;
        _lowCmd->motorCmd[i].tau = (act_temp[i] + act_mean[i] - _lowState->motorState[i].q) * stiffness(i) - _lowState->motorState[i].q * damping(i);
    }
}

void State_locomotion::exit()
{
    counter = 0;
    act_prev.setZero();
    threadRunning = false;
    _thread.join();
}

FSMStateName State_locomotion::checkChange()
{
    if (_lowState->userCmd == UserCommand::L2_B)
    {
        return FSMStateName::PASSIVE;
    }
    else
    {
        return FSMStateName::LOCOMOTION;
    }
}

void State_locomotion::CreateDumpFile(const char* dump_env)
{
    if (dump_env != nullptr && std::string(dump_env) == "1") {
        dump_policy_input = true;
        input_file.open(input_file_path, std::ios::out | std::ios::app);
        if (input_file.is_open()) {
            // write header: counter,timestamp, followed by 0..N-1 columns
            input_file << "counter,timestamp";
            for (int i = 0; i < stacked_input_size; ++i) 
            {
                input_file << ",c" << i;
            }
            input_file << "\n";
            input_file.flush();
        } else {
            std::cerr << "Failed to open dump file: " << input_file_path << std::endl;
            dump_policy_input = false;
        }

        dump_policy_output = true;
        output_file.open(output_file_path, std::ios::out | std::ios::app);
        if (output_file.is_open()) {
            // write header: counter,timestamp, followed by 0..N-1 columns
            output_file << "counter,timestamp";
            for (int i = 0; i < num_actions; ++i) 
            {
                output_file << ",c" << i;
            }
            output_file << "\n";
            output_file.flush();
        } else {
            std::cerr << "Failed to open dump file: " << output_file_path << std::endl;
            dump_policy_output = false;
        }
    }
}
