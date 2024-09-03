#pragma once

#include <gtsam/nonlinear/Values.h>
#include <gtsam/linear/NoiseModel.h>
#include <gtsam/linear/JacobianFactor.h>
#include <gtsam/inference/Factor.h>
#include <gtsam/base/OptionalJacobian.h>

#include <boost/serialization/base_object.hpp>
#include <boost/assign/list_of.hpp>

namespace gtsam
{
    using boost::assign::cref_list_of;
    
    template<class VALUE1, class VALUE2, class VALUE3, class VALUE4, class VALUE5, class VALUE6, class VALUE7, class VALUE8, class VALUE9>
    class NoiseModelFactor9: public NoiseModelFactor {

        public:

        // typedefs for value types pulled from keys
        typedef VALUE1 X1;
        typedef VALUE2 X2;
        typedef VALUE3 X3;
        typedef VALUE4 X4;
        typedef VALUE5 X5;
        typedef VALUE6 X6;
        typedef VALUE6 X7;
        typedef VALUE6 X8;
        typedef VALUE6 X9;

        protected:

        typedef NoiseModelFactor Base;
        typedef NoiseModelFactor9<VALUE1, VALUE2, VALUE3, VALUE4, VALUE5, VALUE6, VALUE7, VALUE8, VALUE9> This;

        public:

        /**
         * Default Constructor for I/O
         */
        NoiseModelFactor9() {}

        /**
         * Constructor
         * @param noiseModel shared pointer to noise model
         * @param j1 key of the first variable
         * @param j2 key of the second variable
         * @param j3 key of the third variable
         * @param j4 key of the fourth variable
         * @param j5 key of the fifth variable
         * @param j6 key of the fifth variable
         */
        NoiseModelFactor6(const SharedNoiseModel& noiseModel, Key j1, Key j2, Key j3, Key j4, Key j5, Key j6, Key j7, Key j8, Key j9) :
            Base(noiseModel, cref_list_of<6>(j1)(j2)(j3)(j4)(j5)(j6)(j7)(j8)(j9)) {}

        ~NoiseModelFactor9() override {}

        /** methods to retrieve keys */
        inline Key key1() const { return keys_[0]; }
        inline Key key2() const { return keys_[1]; }
        inline Key key3() const { return keys_[2]; }
        inline Key key4() const { return keys_[3]; }
        inline Key key5() const { return keys_[4]; }
        inline Key key6() const { return keys_[5]; }
        inline Key key7() const { return keys_[6]; }
        inline Key key8() const { return keys_[7]; }
        inline Key key9() const { return keys_[8]; }

        /** Calls the 9-key specific version of evaluateError, which is pure virtual
         * so must be implemented in the derived class. */
        Vector unwhitenedError(const Values& x, boost::optional<std::vector<Matrix>&> H = boost::none) const override {
            if(this->active(x)) {
            if(H)
                return evaluateError(x.at<X1>(keys_[0]), x.at<X2>(keys_[1]), x.at<X3>(keys_[2]), x.at<X4>(keys_[3]), x.at<X5>(keys_[4]), x.at<X6>(keys_[5]), x.at<X7>(keys_[6]), x.at<X8>(keys_[7]), x.at<X9>(keys_[8]), (*H)[0], (*H)[1], (*H)[2], (*H)[3], (*H)[4], (*H)[5], (*H)[6], (*H)[7], (*H)[8]);
            else
                return evaluateError(x.at<X1>(keys_[0]), x.at<X2>(keys_[1]), x.at<X3>(keys_[2]), x.at<X4>(keys_[3]), x.at<X5>(keys_[4]), x.at<X6>(keys_[5]), x.at<X7>(keys_[6]), x.at<X8>(keys_[7]), x.at<X9>(keys_[8]));
            } else {
            return Vector::Zero(this->dim());
            }
        }

        /**
         *  Override this method to finish implementing a 9-way factor.
         *  If any of the optional Matrix reference arguments are specified, it should compute
         *  both the function evaluation and its derivative(s) in X1 (and/or X2, X3).
         */
        virtual Vector
        evaluateError(const X1&, const X2&, const X3&, const X4&, const X5&, const X6&, const X7&, const X8&, const X9&,
            boost::optional<Matrix&> H1 = boost::none,
            boost::optional<Matrix&> H2 = boost::none,
            boost::optional<Matrix&> H3 = boost::none,
            boost::optional<Matrix&> H4 = boost::none,
            boost::optional<Matrix&> H5 = boost::none,
            boost::optional<Matrix&> H6 = boost::none,
            boost::optional<Matrix&> H7 = boost::none,
            boost::optional<Matrix&> H8 = boost::none,
            boost::optional<Matrix&> H9 = boost::none) const = 0;

        private:

        /** Serialization function */
        friend class boost::serialization::access;
        template<class ARCHIVE>
        void serialize(ARCHIVE & ar, const unsigned int /*version*/) {
            ar & boost::serialization::make_nvp("NoiseModelFactor",
                boost::serialization::base_object<Base>(*this));
        }
    }; // \class NoiseModelFactor9
}