/*
 * OpenVINS: An Open Platform for Visual-Inertial Research
 * Copyright (C) 2018-2022 Patrick Geneva
 * Copyright (C) 2018-2022 Guoquan Huang
 * Copyright (C) 2018-2022 OpenVINS Contributors
 * Copyright (C) 2018-2019 Kevin Eckenhoff
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <https://www.gnu.org/licenses/>.
 */

#ifndef OV_CORE_CAM_PANO_H
#define OV_CORE_CAM_PANO_H

#include "CamBase.h"

namespace ov_core
{

    /**
     * @brief Panorman model pinhole camera model class
     */
    class CamPano : public CamBase
    {

    public:
        /**
         * @brief Default constructor
         * @param width Width of the camera (raw pixels)
         * @param height Height of the camera (raw pixels)
         */
        CamPano(int width, int height) : CamBase(width, height) {}

        ~CamPano() {}

        /**
         * @brief Given a raw uv point, this will undistort it based on the camera matrices into normalized camera coords.
         * @param uv_dist Raw uv coordinate we wish to undistort
         * @return 2d vector of normalized coordinates
         */
        Eigen::Vector3f undistort_f(const Eigen::Vector2f &uv_dist) override
        {
            const float theta = ((uv_dist(0) + 0.5) / _width - 0.5) * (2 * M_PI);
            const float phi = ((uv_dist(1) + 0.5) / _height - 0.5) * M_PI;
            // Construct our return vector
            Eigen::Vector3f pt_out;
            pt_out(0) = std::cos(phi) * std::sin(theta);
            pt_out(1) = std::sin(phi);
            pt_out(2) = std::cos(phi) * std::cos(theta);
            return pt_out;
        }

        /**
         * @brief Given a normalized uv coordinate this will distort it to the raw image plane
         * @param uv_norm Normalized coordinates we wish to distort
         * @return 2d vector of raw uv coordinate
         */
        Eigen::Vector2f distort_f(const Eigen::Vector3f &uv_norm) override
        {
            float l_n = uv_norm.norm();
            if (l_n < 0.1)
                return Eigen::Vector2f(-1, -1);

            const float theta = std::atan2(uv_norm(0), uv_norm(2));
            const float phi = std::asin(uv_norm(1) / uv_norm.norm());
            // Calculate distorted coordinates
            Eigen::Vector2f uv_dist(_width * (0.5 + theta / (2 * M_PI)), _height * (0.5 + phi / M_PI));
            return uv_dist;
        }

        /**
         * @brief Computes the derivative of raw distorted to normalized coordinate.
         * @param uv_norm Normalized coordinates we wish to distort
         * @param H_dz_dzn Derivative of measurement z in respect to normalized
         * @param H_dz_dzeta Derivative of measurement z in respect to intrinic parameters
         */

        void compute_distort_jacobian(const Eigen::Vector3d &p_FinCi, Eigen::MatrixXd &H_dz_dpfc, Eigen::MatrixXd &H_dz_dzeta) override
        {
            double l_n = p_FinCi.norm();

            Eigen::MatrixXd dz_dtheta(2, 2);
            dz_dtheta << _width / (2 * M_PI), 0, 0, _height / M_PI;
            Eigen::MatrixXd dtheta_dpfc(2, 3);
            const double s1 = p_FinCi(0) * p_FinCi(0) + p_FinCi(2) * p_FinCi(2);
            const double s2 = l_n * l_n * std::sqrt(s1);
            dtheta_dpfc << p_FinCi(2) / s1, 0, -p_FinCi(0) / s1, -p_FinCi(0) * p_FinCi(1) / s2, s1 / s2, -p_FinCi(1) * p_FinCi(2) / s2;

            H_dz_dpfc = dz_dtheta * dtheta_dpfc;

            H_dz_dzeta = Eigen::MatrixXd::Zero(2, 8);
        }
    };

} // namespace ov_core

#endif /* OV_CORE_CAM_EQUI_H */