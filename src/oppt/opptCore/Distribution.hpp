/**
 * Copyright 2017
 *
 * This file is part of On-line POMDP Planning Toolkit (OPPT).
 * OPPT is free software: you can redistribute it and/or modify it under the terms of the
 * GNU General Public License published by the Free Software Foundation,
 * either version 2 of the License, or (at your option) any later version.
 *
 * OPPT is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY;
 * without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along with OPPT.
 * If not, see http://www.gnu.org/licenses/.
 */
#ifndef __OPPT_DISTRIBUTION_HPP__
#define __OPPT_DISTRIBUTION_HPP__
#include "oppt/opptCore/core.hpp"

using namespace Eigen;

namespace oppt
{

/**
 * Base class for a distribution.
 */
template<typename Scalar>
class Distribution
{
public:
    _NO_COPY_BUT_MOVE(Distribution)
    /**
     * @brief Construct from a oppt::RandomEnginePtr
     */
    Distribution(RandomEnginePtr& randomEngine):
        randomEngine_(randomEngine) {
    }

    virtual ~Distribution() = default;

    _STATIC_CAST

    /**
     * Draws samples from the distribution.
     * @param numSamples The number of samples to draw
     * @return A dynamically sized Eigen::Matrix, where the colums are the samples drawn from the distribution.
     */
    virtual const Eigen::Matrix < Scalar, Dynamic, -1 > sample(const unsigned int& numSamples) const = 0;

    /**
     * Evaluate the density function (PDF) at a particular position. This method should only be implemented
     * when the PDF can actually be evaluated
     * @param position The point at which the PDF is evaluated
     * @return The PDF at the evaluated position
     */
    virtual FloatType pdf(const std::vector<Scalar>& position) const {
        return -1.0;
    }

    /**
     * @brief Get the number of dimensions of the distribution
     */
    virtual unsigned int getNumDimensions() const = 0;

    virtual void setRandomEngine(RandomEnginePtr &randomEngine) {
        randomEngine_ = randomEngine;
    }

protected:
    RandomEnginePtr randomEngine_;

};

/**
 * Uniform distribution over an interval [left, right]
 */
template<typename Scalar>
class UniformDistribution: public Distribution<Scalar>
{
public:
    /**
     * Constructs the uniform distribution from left and right bounding vectors and
     * a oppt::RandomEnginePtr
     */
    UniformDistribution(const std::vector<Scalar>& left,
                        const std::vector<Scalar>& right,
                        RandomEnginePtr& randomEngine):
        Distribution<Scalar>(randomEngine),
        left_(left),
        right_(right),
        volume_(1),
        numDimensions_(left.size()) {
        for (size_t i = 0; i != left.size(); ++i) {
            volume_ *= std::fabs(right[i] - left[i]);
        }

    }
    
    UniformDistribution(const UniformDistribution& other):
        Distribution<Scalar>(other.randomEngine_),
        left_(other.left_),
        right_(other.right_),
        volume_(other.volume_),
        numDimensions_(other.numDimensions_) {}

    UniformDistribution& operator=(const UniformDistribution& other) {
        if (this != &other) {
            left_ = other.left_;
            right_ = other.right_;
            volume_ = other.volume_;
            numDimensions_ = other.numDimensions_;
        }

        return *this;
    }

    virtual const Eigen::Matrix < Scalar, Dynamic, -1 > sample(const unsigned int& numSamples) const override {
        Matrix < Scalar, Dynamic, Dynamic > samples(numDimensions_, numSamples);
        for (size_t i = 0; i != numSamples; ++i) {
            Eigen::Matrix<Scalar, Dynamic, 1> colVec(numDimensions_);
            for (size_t j = 0; j != numDimensions_; ++j) {
                std::uniform_real_distribution<Scalar> dist(left_[j], right_[j]);
                auto sample = dist(*(this->randomEngine_.get()));
                colVec(j) = sample;
            }

            samples.col(i) = colVec;
        }

        return samples;
    }

    virtual FloatType pdf(const std::vector<Scalar>& point) const override {
        if (point.size() != left_.size())
            return 0.0;
        for (size_t i = 0; i != point.size(); ++i) {
            if (point[i] < left_[i] || point[i] > right_[i])
                return 0.0;
        }
        return 1.0 / volume_;
    }

    virtual unsigned int getNumDimensions() const override {
        return numDimensions_;
    }

private:
    Scalar volume_;

    std::vector<Scalar> left_;

    std::vector<Scalar> right_;

    unsigned int numDimensions_;
};

/**
 * Mutivariate normal distribution
 */
template<typename Scalar>
class MultivariateNormalDistribution: public Distribution<Scalar>
{
public:
    MultivariateNormalDistribution(RandomEnginePtr& randomEngine):
        Distribution<Scalar>(randomEngine) {
        randN_.setRandomEngine(this->randomEngine_);
    }

    // Copy constructor
    MultivariateNormalDistribution(const MultivariateNormalDistribution& other):
        Distribution<Scalar>(other.randomEngine_),
        mean_(other.mean_),
        covariance_(other.covariance_),
        randN_(other.randN_) {

    }

    // Assignment operator
    MultivariateNormalDistribution& operator=(const MultivariateNormalDistribution& other) {
        if (this != &other) {
            randN_ = other.randN_;
            mean_ = other.mean_;
            covariance_ = other.covariance_;
        }

        return *this;
    }
    
    /**
     * @brief Set the mean of the distribution
     */
    void setMean(const Eigen::Matrix< Scalar, Dynamic, 1>& mean) {
        mean_ = mean;
    }

    /**
     * @brief Set the covariance matrix of the distribution
     */
    void setCovariance(const Eigen::Matrix<Scalar, Dynamic, Dynamic>& covariance) {
        covariance_ = covariance;
        covarianceInverse_ = covariance_.inverse();

        eigenSolver_ = SelfAdjointEigenSolver<Matrix<Scalar, Dynamic, Dynamic> >(covariance_);
        transform_ = eigenSolver_.eigenvectors() * eigenSolver_.eigenvalues().cwiseMax(0).cwiseSqrt().asDiagonal();

        //denominator_ = sqrt((2.0 * M_PI * covariance_).determinant());
        denominator_ = sqrt(std::pow(2.0 * M_PI, covariance_.rows()) * covariance_.determinant());
    }

    virtual FloatType pdf(const std::vector<Scalar>& position) const override {
        Matrix<Scalar, Dynamic, 1> x = Matrix<Scalar, Dynamic, 1>::Map(position.data(), position.size());
        Matrix<Scalar, Dynamic, 1> xMu(x - mean_);
        return std::exp((-1.0 / 2.0) * xMu.transpose() * covarianceInverse_ * xMu) / denominator_;
    }

    virtual const Eigen::Matrix < Scalar, Dynamic, -1 > sample(const unsigned int& numSamples) const override {
        return (transform_ * Matrix < Scalar, Dynamic, -1 >::NullaryExpr(covariance_.rows(), numSamples, randN_)).colwise() + mean_;
    }

    /**
     * @brief Get the covariance matrix
     */
    Eigen::Matrix<Scalar, Dynamic, Dynamic> getCovarianceMatrix() const {
        return covariance_;
    }
    
    /**
     * @brief Get the mean
     */
    Eigen::Matrix< Scalar, Dynamic, 1> getMean() const {
        mean_;
    }

    virtual unsigned int getNumDimensions() const override {
        return mean_.rows();
    }

    virtual void setRandomEngine(RandomEnginePtr &randomEngine) override {
        this->randomEngine_ = randomEngine;
        randN_.setRandomEngine(this->randomEngine_);
    }

private:
    struct ScalarDistOperator {
    public:
        mutable std::normal_distribution<Scalar> norm;

        mutable std::uniform_real_distribution<FloatType> real_distr;

        ScalarDistOperator() {

        }

        template<typename Index>
        inline const Scalar operator()(Index, Index = 0) const {
            return norm(*(randomEngine_.get()));
        }

        inline FloatType sampleUniform() {
            FloatType sample = real_distr(*(randomEngine_.get()));
            return sample;
        }

        inline void setRandomEngine(RandomEnginePtr randomEngine) {
            randomEngine_ = randomEngine;
        }

    private:
        RandomEnginePtr randomEngine_;

    };

    ScalarDistOperator randN_;

private:
    Eigen::Matrix< Scalar, Dynamic, 1> mean_;

    Eigen::Matrix<Scalar, Dynamic, Dynamic> covariance_;

    Eigen::Matrix<Scalar, Dynamic, Dynamic> covarianceInverse_;

    Scalar denominator_;

    SelfAdjointEigenSolver<Matrix<Scalar, Dynamic, Dynamic> > eigenSolver_;

    Matrix<Scalar, Dynamic, Dynamic> transform_;

};

}

#endif
