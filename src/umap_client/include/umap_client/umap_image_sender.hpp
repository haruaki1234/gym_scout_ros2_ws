/**
 * @file umap_image_sender.hpp
 * @author Takuma Nakao
 * @brief UMap画像送信クライアント
 * @date 2024-05-23
 *
 * @copyright Copyright (c) 2024
 *
 */
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

/**
 * @brief VGM画像送信クラス
 *
 */
class UmapImageSender {
public:
    /**
     * @brief 位置姿勢
     *
     */
    struct pose_t {
        float x = 0;     //!< 位置X
        float y = 0;     //!< 位置Y
        float z = 0;     //!< 位置Z
        float roll = 0;  //!< 姿勢Roll
        float pitch = 0; //!< 姿勢Pitch
        float yaw = 0;   //!< 姿勢Yaw
        /**
         * @brief Construct a new pose t object
         *
         * @param _x 位置X
         * @param _y 位置Y
         * @param _z 位置Z
         * @param _roll 姿勢Roll
         * @param _pitch 姿勢Pitch
         * @param _yaw 姿勢Yaw
         */
        pose_t(float _x = 0, float _y = 0, float _z = 0, float _roll = 0, float _pitch = 0, float _yaw = 0) : x(_x), y(_y), z(_z), roll(_roll), pitch(_pitch), yaw(_yaw) {}
    };
    /**
     * @brief VGM推定結果
     *
     */
    struct result_t {
        pose_t pose;            //!< 位置姿勢
        float similar;          //!< 類似度
        float similar_average;  //!< 類似度平均
        float similar_variance; //!< 類似度分散
    };
    /**
     * @brief 線検出タイプ
     *
     */
    enum class LineDetectionType { CANNY, LSD, FLD };

    //! 推定結果を返送してもらうか
    bool is_return = true;
    //! キャリブレーション済みか
    bool is_calibrated = true;
    //! リサイズ済みか
    bool is_resized = true;
    //! 線検出済みか
    bool is_line_detected = true;
    //! 線検出タイプ
    LineDetectionType line_detection_type = LineDetectionType::CANNY;

    //! タイムアウト時間[ms]
    int dead_time_ms = 1000;

private:
    //! サーバアドレス
    std::string address_ = "";
    //! サーバポート
    int port_ = 0;
    //! デバイスID
    std::string device_id_ = "";
    //! カメラ番号
    int camera_number_ = 0;

    /**
     * @brief ファイル名の作成
     *
     * @return std::string ファイル名
     */
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
        auto now = std::chrono::system_clock::now();
        auto time = std::chrono::system_clock::to_time_t(now);
        auto time_info = std::localtime(&time);
        int year = time_info->tm_year + 1900;
        file_name << std::setw(2) << std::setfill('0') << std::to_string(year % ((year / 1000) * 1000));
        file_name << std::setw(2) << std::setfill('0') << std::to_string(time_info->tm_mon + 1);
        file_name << std::setw(2) << std::setfill('0') << std::to_string(time_info->tm_mday);
        file_name << std::setw(2) << std::setfill('0') << std::to_string(time_info->tm_hour);
        file_name << std::setw(2) << std::setfill('0') << std::to_string(time_info->tm_min);
        file_name << std::setw(2) << std::setfill('0') << std::to_string(time_info->tm_sec);
        file_name << std::setw(4) << std::setfill('0') << std::to_string(std::chrono::duration_cast<std::chrono::milliseconds>(now.time_since_epoch()).count() % 1000);
        // file_name << std::setw(4) << std::setfill('0') << device_id_;
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

public:
    /**
     * @brief Construct a new Umap Image Sender object
     *
     * @param address サーバアドレス
     * @param port サーバポート
     * @param device_id デバイスID
     * @param camera_number カメラ番号
     */
    UmapImageSender(const std::string& address, int port, const std::string& device_id, int camera_number) : address_(address), port_(port), device_id_(device_id), camera_number_(camera_number) {}
    /**
     * @brief cv::Mat画像を送信する
     *
     * @param img クエリ画像
     * @param pose 推定初期位置
     * @param matching_distance マッチング位置範囲
     * @param matching_angle_distance マッチング角度範囲
     * @return std::optional<result_t> 推定結果
     */
    std::optional<result_t> send_from_mat(const cv::Mat& img, const pose_t& pose, float matching_distance, float matching_angle_distance)
    {
        using namespace std::chrono_literals;
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

        asio::io_context io_context;
        asio::ip::tcp::socket socket(io_context);

        asio::system_timer timer(io_context);
        timer.expires_from_now(std::chrono::milliseconds(dead_time_ms));
        timer.async_wait([&](const std::error_code& ec) {
            if (!ec) {
                std::cout << "timeout" << std::endl;
                if (socket.is_open()) {
                    socket.close();
                }
            }
        });

        std::optional<result_t> result = std::nullopt;
        asio::streambuf receive_buffer;

        socket.async_connect(asio::ip::tcp::endpoint(asio::ip::address_v4::from_string(address_), port_), [&](const std::error_code& ec) {
            if (ec) {
                if (ec != asio::error::operation_aborted) {
                    std::cout << "connect failed : " << ec.message() << std::endl;
                }
                timer.cancel();
            }
            else {
                asio::async_write(socket, asio::buffer(binary, binary.size()), [&](const std::error_code& ec, size_t bytes_transferred) {
                    if (ec) {
                        if (ec != asio::error::operation_aborted) {
                            std::cout << "send failed: " << ec.message() << std::endl;
                        }
                        timer.cancel();
                    }
                    else {
                        if (is_return) {
                            asio::async_read(socket, receive_buffer, asio::transfer_all(), [&](const std::error_code& ec, size_t length) {
                                if (ec && ec != asio::error::eof) {
                                    if (ec != asio::error::operation_aborted) {
                                        std::cout << "receive failed : " << ec.message() << std::endl;
                                    }
                                    timer.cancel();
                                }
                                else {
                                    timer.cancel();
                                    result = result_t();
                                    std::array<float, 9> float_array;
                                    std::memcpy(float_array.data(), asio::buffer_cast<const uint8_t*>(receive_buffer.data()), sizeof(float) * 9);
                                    result->pose.x = float_array[0];
                                    result->pose.y = float_array[1];
                                    result->pose.z = float_array[2];
                                    result->pose.roll = float_array[3];
                                    result->pose.pitch = float_array[4];
                                    result->pose.yaw = float_array[5];
                                    result->similar = float_array[6];
                                    result->similar_average = float_array[7];
                                    result->similar_variance = float_array[8];
                                    receive_buffer.consume(receive_buffer.size());
                                }
                            });
                        }
                        else {
                            timer.cancel();
                        }
                    }
                });
            }
        });

        io_context.run();
        if (socket.is_open()) {
            socket.close();
        }
        return result;
    }
};

} // namespace umap