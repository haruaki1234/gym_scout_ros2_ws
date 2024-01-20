#pragma once

#include <string>
#include <chrono>
#include <sstream>
#include <iomanip>

#include <opencv2/opencv.hpp>

#define ASIO_DISABLE_NOEXCEPT 1
#include <asio/asio/include/asio.hpp>

namespace umap
{

class UmapImageSender {
public:
    struct pose_t {
        float x = 0;
        float y = 0;
        float z = 0;
        float roll = 0;
        float pitch = 0;
        float yaw = 0;
        pose_t(float _x = 0, float _y = 0, float _z = 0, float _roll = 0, float _pitch = 0, float _yaw = 0) : x(_x), y(_y), z(_z), roll(_roll), pitch(_pitch), yaw(_yaw) {}
    };
    struct result_t {
        pose_t pose;
        float similar;
        float similar_average;
        float similar_variance;
    };
    enum class LineDetectionType { CANNY, LSD, FLD };

    bool is_return = true;
    bool is_calibrated = true;
    bool is_resized = true;
    bool is_line_detected = true;
    LineDetectionType line_detection_type = LineDetectionType::CANNY;

private:
    std::string address_ = "";
    int port_ = 0;
    std::string device_id_ = "";
    int camera_number_ = 0;
    asio::io_context io_context_;

    std::string make_file_name() const
    {
        std::stringstream file_name;
        if (is_return) {
            file_name << "T";
        }
        else {
            file_name << "F";
        }
        file_name << "R";
        auto time = std::chrono::system_clock::to_time_t(std::chrono::system_clock::now());
        auto time_info = std::localtime(&time);
        int year = time_info->tm_year + 1900;
        file_name << std::setw(2) << std::setfill('0') << std::to_string(year % ((year / 1000) * 1000));
        file_name << std::setw(2) << std::setfill('0') << std::to_string(time_info->tm_mon + 1);
        file_name << std::setw(2) << std::setfill('0') << std::to_string(time_info->tm_mday);
        file_name << std::setw(2) << std::setfill('0') << std::to_string(time_info->tm_hour);
        file_name << std::setw(2) << std::setfill('0') << std::to_string(time_info->tm_min);
        file_name << std::setw(2) << std::setfill('0') << std::to_string(time_info->tm_sec);
        file_name << std::setw(4) << std::setfill('0') << device_id_;
        file_name << std::setw(2) << std::setfill('0') << std::to_string(camera_number_);
        std::string process_histry = "";
        if (is_calibrated) {
            process_histry += "c";
        }
        else {
            process_histry += "C";
        }
        if (is_resized) {
            process_histry += "r";
        }
        else {
            process_histry += "R";
        }
        if (is_line_detected) {
            switch (line_detection_type) {
            case LineDetectionType::CANNY:
                process_histry += "n";
                break;
            case LineDetectionType::LSD:
                process_histry += "l";
                break;
            case LineDetectionType::FLD:
                process_histry += "f";
                break;
            }
        }
        else {
            switch (line_detection_type) {
            case LineDetectionType::CANNY:
                process_histry += "N";
                break;
            case LineDetectionType::LSD:
                process_histry += "L";
                break;
            case LineDetectionType::FLD:
                process_histry += "F";
                break;
            }
        }
        file_name << std::setw(8) << std::setfill('0') << process_histry;
        file_name << ".png";
        return file_name.str();
    }
    void send_binary(asio::ip::tcp::socket& socket, const std::vector<uint8_t>& binary)
    {
        using asio::ip::tcp;
        try {
            socket.connect(tcp::endpoint(asio::ip::address_v4::from_string(address_), port_));
            io_context_.run();
            asio::write(socket, asio::buffer(binary, binary.size()));
        }
        catch (const std::exception& e) {
            std::cout << e.what() << std::endl;
        }
    }
    result_t receive(asio::ip::tcp::socket& socket)
    {
        using asio::ip::tcp;
        // tcp::acceptor acceptor(io_context_, tcp::endpoint(tcp::v4(), port_));
        // tcp::socket socket(io_context_);
        asio::streambuf receive_buffer;
        std::error_code error;
        // acceptor.accept(socket);
        // asio::socket_base::keep_alive option(true);
        // socket.set_option(option);

        result_t result;
        asio::read(socket, receive_buffer, asio::transfer_all(), error);
        if (error && error != asio::error::eof) {
            std::cout << "receive failed : " << error.message() << std::endl;
        }
        else {
            std::array<float, 9> float_array;
            std::memcpy(float_array.data(), asio::buffer_cast<const uint8_t*>(receive_buffer.data()), sizeof(float) * 9);
            result.pose.x = float_array[0];
            result.pose.y = float_array[1];
            result.pose.z = float_array[2];
            result.pose.roll = float_array[3];
            result.pose.pitch = float_array[4];
            result.pose.yaw = float_array[5];
            result.similar = float_array[6];
            result.similar_average = float_array[7];
            result.similar_variance = float_array[8];
            receive_buffer.consume(receive_buffer.size());
        }
        socket.close();
        return result;
    }

public:
    UmapImageSender(const std::string& address, int port, const std::string& device_id, int camera_number) : address_(address), port_(port), device_id_(device_id), camera_number_(camera_number) {}
    std::optional<result_t> send_from_mat(const cv::Mat& img, const pose_t& pose, float matching_distance, float matching_angle_distance)
    {
        auto file_name = make_file_name();
        std::vector<uint8_t> file_name_binary(file_name.begin(), file_name.end());
        std::vector<uint8_t> img_binary;
        cv::imencode(".png", img, img_binary);
        std::array<float, 8> float_array = {pose.x, pose.y, pose.z, pose.roll, pose.pitch, pose.yaw, matching_distance, matching_angle_distance};
        std::vector<uint8_t> float_binary(sizeof(float) * 8);
        std::memcpy(float_binary.data(), float_array.data(), sizeof(float) * 8);
        std::vector<uint8_t> binary;
        binary.insert(binary.end(), file_name_binary.begin(), file_name_binary.end());
        binary.insert(binary.end(), img_binary.begin(), img_binary.end());
        binary.insert(binary.end(), float_binary.begin(), float_binary.end());
        uint32_t data_size = binary.size();
        std::vector<uint8_t> data_size_binary(sizeof(uint32_t));
        std::memcpy(data_size_binary.data(), &data_size, sizeof(uint32_t));
        binary.insert(binary.begin(), data_size_binary.begin(), data_size_binary.end());
        asio::ip::tcp::socket socket(io_context_);
        send_binary(socket, binary);
        if (is_return) {
            return receive(socket);
        }
        else {
            socket.close();
            return std::nullopt;
        }
    }
};

} // namespace umap