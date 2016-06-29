//
// Copyright (c) 2013, 2014 CNRS
// Author: Florent Lamiraux
//
//
// This file is part of hpp-model
// hpp-model is free software: you can redistribute it
// and/or modify it under the terms of the GNU Lesser General Public
// License as published by the Free Software Foundation, either version
// 3 of the License, or (at your option) any later version.
//
// hpp-model is distributed in the hope that it will be
// useful, but WITHOUT ANY WARRANTY; without even the implied warranty
// of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
// General Lesser Public License for more details.  You should have
// received a copy of the GNU Lesser General Public License along with
// hpp-model  If not, see
// <http://www.gnu.org/licenses/>.

#include <limits>
#include <stdexcept>
#include <cstdlib>
#include <iostream>
#include <sstream>
#include <Eigen/Geometry>
#include <hpp/fcl/math/transform.h>
#include <hpp/util/debug.hh>
#include <hpp/model/joint-configuration.hh>

namespace hpp {
  namespace model {
    typedef Eigen::AngleAxis <value_type> AngleAxis_t;
    typedef Eigen::Quaternion <value_type> Quaternion_t;

    JointConfiguration::JointConfiguration (size_type configSize)
    {
      bounded_.resize (configSize);
      lowerBounds_.resize (configSize);
      upperBounds_.resize (configSize);
      for (size_type i=0; i < (size_type)bounded_.size (); ++i) {
	bounded_ [i] = false;
	lowerBounds_ [i] = -std::numeric_limits<value_type>::infinity();
	upperBounds_ [i] = +std::numeric_limits<value_type>::infinity();
      }
    }

    JointConfiguration::~JointConfiguration ()
    {
    }

    void JointConfiguration::isBounded (size_type rank, bool bounded)
    {
      bounded_ [rank] = bounded;
    }

    bool JointConfiguration::isBounded (size_type rank) const
    {
      return bounded_ [rank];
    }

    value_type JointConfiguration::lowerBound (size_type rank) const
    {
      return lowerBounds_ [rank];
    }

    value_type JointConfiguration::upperBound (size_type rank) const
    {
      return upperBounds_ [rank];
    }

    void JointConfiguration::lowerBound (size_type rank, value_type lowerBound)
    {
      lowerBounds_ [rank] = lowerBound;
    }

    void JointConfiguration::upperBound (size_type rank, value_type upperBound)
    {
      upperBounds_ [rank] = upperBound;
    }

    AnchorJointConfig::AnchorJointConfig () : JointConfiguration (0)
    {
    }

    AnchorJointConfig::~AnchorJointConfig ()
    {
    }

    SO3JointConfig::SO3JointConfig () : JointConfiguration (4)
    {
      for (size_type i=0; i<4; ++i) {
	lowerBound (i, -1);
	upperBound (i, 1);
      }
    }

    SO3JointConfig::~SO3JointConfig ()
    {
    }

    template <size_type dimension>
    TranslationJointConfig <dimension>::TranslationJointConfig () :
      JointConfiguration (dimension)
    {
    }

    template <size_type dimension>
    TranslationJointConfig <dimension>::~TranslationJointConfig ()
    {
    }

    void AnchorJointConfig::interpolate (ConfigurationIn_t,
					 ConfigurationIn_t,
					 const value_type&,
					 const size_type&,
					 ConfigurationOut_t)
    {
    }

    value_type AnchorJointConfig::squaredDistance (ConfigurationIn_t,
					           ConfigurationIn_t,
					           const size_type&) const
    {
      return 0;
    }

    value_type AnchorJointConfig::distance (ConfigurationIn_t,
					    ConfigurationIn_t,
					    const size_type&) const
    {
      return 0;
    }

    void AnchorJointConfig::integrate (ConfigurationIn_t,
				       vectorIn_t,
				       const size_type&,
				       const size_type&,
				       ConfigurationOut_t) const
    {
    }

    void AnchorJointConfig::difference (ConfigurationIn_t,
					ConfigurationIn_t,
					const size_type&,
					const size_type&,
					vectorOut_t) const
    {
    }

    bool AnchorJointConfig::isApprox (ConfigurationIn_t q1,
				      ConfigurationIn_t q2,
				      const size_type& indexConfig,
				      const value_type& eps) const
    {
      return true;
    }

    /// Normalize configuration of joint
    void AnchorJointConfig::normalize (const size_type&,
				       ConfigurationOut_t) const
    {
    }

    void AnchorJointConfig::uniformlySample (const size_type&,
					     ConfigurationOut_t) const
    {
    }

    /// Compute quaternion and angle from a SO(3) joint configuration
    /// \note this angle corresponds to the angle between q1 and q2, in \f$ \mathbb{R}^4 \f$.
    ///       The angle of the rotation going from q1 to q2 is half this angle.
    ///
    /// \param q1, q2, robot configurations
    /// \param index index of joint configuration in robot configuration vector
    /// \param unit quaternion corresponding to both joint configuration
    /// \return angle between both joint configuration
    static value_type angleBetweenQuaternions (ConfigurationIn_t q1,
					       ConfigurationIn_t q2,
					       const size_type& index,
                                               bool& cosIsNegative)
    {
      if (q1.segment (index, 4) == q2.segment (index, 4)) {
	cosIsNegative = false;
	return 0;
      }
      value_type innerprod = q1.segment (index, 4).dot (q2.segment (index, 4));
      assert (fabs (innerprod) < 1.0001);
      if (innerprod < -1) innerprod = -1;
      if (innerprod >  1) innerprod =  1;
      cosIsNegative = (innerprod < 0);
      value_type theta = acos (innerprod);
      return theta;
    }

    void SO3JointConfig::interpolate (ConfigurationIn_t q1,
				      ConfigurationIn_t q2,
				      const value_type& u,
				      const size_type& index,
				      ConfigurationOut_t result)
    {
      // for rotation part, transform roll pitch yaw into quaternion
      bool cosIsNegative;
      const value_type angle = angleBetweenQuaternions (q1, q2, index, cosIsNegative);
      const int invertor = (cosIsNegative) ? -1 : 1;
      const value_type theta = (cosIsNegative) ? (M_PI - angle) : angle;

      // theta is between 0 and M_PI/2.
      // sin(alpha*theta)/sin(theta) in 0 should be computed differently.
      if (fabs (theta) > 1e-6) {
        const value_type sintheta_inv = 1 / sin (theta);
	result.segment (index, 4) =
	  (sin ((1-u)*theta) * sintheta_inv)  * q1.segment (index, 4) +
	  invertor * (sin (u*theta) * sintheta_inv) * q2.segment (index, 4);
	assert (result.segment (index, 4).norm () ==
		result.segment (index, 4).norm ());
      } else {
	result.segment (index, 4) =
	  (1-u) * q1.segment (index, 4) + invertor * u * q2.segment (index, 4);
	assert (result.segment (index, 4).norm () ==
		result.segment (index, 4).norm ());
      }
    }

    value_type SO3JointConfig::squaredDistance (ConfigurationIn_t q1,
                                                ConfigurationIn_t q2,
                                                const size_type& index) const
    {
      const value_type d (distance (q1, q2, index));
      return d*d;
    }

    value_type SO3JointConfig::distance (ConfigurationIn_t q1,
					 ConfigurationIn_t q2,
					 const size_type& index) const
    {
      bool cosIsNegative;
      value_type theta = angleBetweenQuaternions (q1, q2, index, cosIsNegative);
      assert (theta >= 0);
      assert (M_PI - theta >= 0);
      return 2 * (cosIsNegative ? M_PI - theta : theta);
    }

    void SO3JointConfig::integrate (ConfigurationIn_t q,
				    vectorIn_t v,
				    const size_type& indexConfig,
				    const size_type& indexVelocity,
				    ConfigurationOut_t result) const
    {
      vector3_t omega (v [indexVelocity + 0], v [indexVelocity + 1],
		       v [indexVelocity + 2]);

      value_type angle = .5*omega.norm();
      if (angle == 0) {
	result.segment<4> (indexConfig) = q.segment <4> (indexConfig);
	return;
      }
      // try to keep norm of quaternion close to 1.
      value_type norm2p = q.segment <4> (indexConfig).squaredNorm ();
      Quaternion_t p ((1.5-.5*norm2p) * q [indexConfig + 0],
		      (1.5-.5*norm2p) * q [indexConfig + 1],
		      (1.5-.5*norm2p) * q [indexConfig + 2],
		      (1.5-.5*norm2p) * q [indexConfig + 3]);
      vector3_t k = (sin (angle)/omega.norm())*omega;
      Quaternion_t pOmega (cos (angle), k [0], k [1], k [2]);
      Quaternion_t res = pOmega*p;
      // Eigen does not allow to get the 4 coefficients at once.
      result [indexConfig + 0] = res.w ();
      result [indexConfig + 1] = res.x ();
      result [indexConfig + 2] = res.y ();
      result [indexConfig + 3] = res.z ();
    }

    void SO3JointConfig::difference (ConfigurationIn_t q1,
				     ConfigurationIn_t q2,
				     const size_type& indexConfig,
				     const size_type& indexVelocity,
				     vectorOut_t result) const
    {
      if (q1.segment <4> (indexConfig) == q2.segment <4> (indexConfig)) {
	result.segment <3> (indexVelocity).setZero ();
	return;
      }
      const int invertor =
        (q1.segment <4> (indexConfig)
         .dot (q2.segment <4> (indexConfig)) < 0
         ) ? -1 : 1;

      // Compute rotation vector between q2 and q1.
      // Cannot use the following because eigen convention is (x,y,z,w) when
      // passing a 4d vector...
      // Quaternion_t p1 (invertor * q1.segment <4> (indexConfig));
      // Quaternion_t p2 (q2.segment <4> (indexConfig));
      Quaternion_t p1 (invertor * q1 [indexConfig + 0],
                       invertor * q1 [indexConfig + 1],
                       invertor * q1 [indexConfig + 2],
                       invertor * q1 [indexConfig + 3]);
      Quaternion_t p2 (q2 [indexConfig + 0],
                       q2 [indexConfig + 1],
                       q2 [indexConfig + 2],
                       q2 [indexConfig + 3]);
      Quaternion_t p (p1*p2.conjugate ());
      AngleAxis_t angleAxis (p);
      result.segment <3> (indexVelocity) = angleAxis.angle () * angleAxis.axis ();
    }

    bool SO3JointConfig::isApprox (ConfigurationIn_t q1, ConfigurationIn_t q2,
				   const size_type& indexConfig,
				   const value_type& eps) const
    {
      return (q1.segment <4> (indexConfig).isApprox
	      (q2.segment <4> (indexConfig)) ||
	      q1.segment <4> (indexConfig).isApprox
	      (-q2.segment <4> (indexConfig)));
    }

    /// Normalize configuration of joint
    void SO3JointConfig::normalize (const size_type& index,
				    ConfigurationOut_t result) const
    {
      result.segment <4> (index).normalize ();
    }

    void SO3JointConfig::uniformlySample (const size_type& index,
					  ConfigurationOut_t result) const
    {
      value_type u1 = (value_type)rand() / RAND_MAX;
      value_type u2 = (value_type)rand() / RAND_MAX;
      value_type u3 = (value_type)rand() / RAND_MAX;
      result [index] = sqrt (1-u1)*sin(2*M_PI*u2);
      result [index+1] = sqrt (1-u1)*cos(2*M_PI*u2);
      result [index+2] = sqrt (u1) * sin(2*M_PI*u3);
      result [index+3] = sqrt (u1) * cos(2*M_PI*u3);
    }

    template <size_type dimension>
    void TranslationJointConfig <dimension>::interpolate
    (ConfigurationIn_t q1, ConfigurationIn_t q2, const value_type& u,
     const size_type& index, ConfigurationOut_t result)
    {
      result.segment <dimension> (index) =
	(1-u) * q1.segment <dimension> (index) +
	u * q2.segment <dimension> (index);
    }

    template <size_type dimension>
    value_type TranslationJointConfig <dimension>::squaredDistance
    (ConfigurationIn_t q1, ConfigurationIn_t q2, const size_type& index) const
    {
      return (q2.segment <dimension> (index) - q1.segment <dimension> (index)).squaredNorm ();
    }

    template <size_type dimension>
    value_type TranslationJointConfig <dimension>::distance
    (ConfigurationIn_t q1, ConfigurationIn_t q2, const size_type& index) const
    {
      if (dimension == 1) {
	return fabs (q2 [index] - q1 [index]);
      } else {
	return (q2.segment <dimension> (index) -
		q1.segment <dimension> (index)).norm ();
      }
    }

    template <size_type dimension>
    void TranslationJointConfig <dimension>::integrate
    (ConfigurationIn_t q, vectorIn_t v, const size_type& indexConfig,
     const size_type& indexVelocity, ConfigurationOut_t result) const
    {
      assert (indexConfig < result.size ());
      result.segment <dimension> (indexConfig) =
	q.segment <dimension> (indexConfig) +
	v.segment <dimension> (indexVelocity);
      for (unsigned int i=0; i<dimension; ++i) {
	if (isBounded (i)) {
	  if (result [indexConfig + i] < lowerBound (i)) {
	    result [indexConfig + i] = lowerBound (i);
	  } else if (result [indexConfig + i] > upperBound (i)) {
	    result [indexConfig + i] = upperBound (i);
	  }
	}
      }
    }

    template <size_type dimension>
    void TranslationJointConfig <dimension>::difference
    (ConfigurationIn_t q1, ConfigurationIn_t q2, const size_type& indexConfig,
     const size_type& indexVelocity, vectorOut_t result) const
    {
      result.segment <dimension> (indexVelocity) =
	q1.segment <dimension> (indexConfig) -
	q2.segment <dimension> (indexConfig);
    }

    template <size_type dimension>
    bool TranslationJointConfig <dimension>::isApprox
    (ConfigurationIn_t q1, ConfigurationIn_t q2,
     const size_type& indexConfig, const value_type& eps) const
    {
      return q1.segment <dimension> (indexConfig).isApprox
	(q2.segment <dimension> (indexConfig));
    }

    /// Normalize configuration of joint
    /// Do nothing
    template <size_type dimension>
    void TranslationJointConfig <dimension>::normalize
    (const size_type&, ConfigurationOut_t) const
    {
    }

    template <size_type dimension>
    void TranslationJointConfig <dimension>::uniformlySample
    (const size_type& index, ConfigurationOut_t result) const
    {
      for (unsigned int i=0; i<dimension; ++i) {
	if (!isBounded (i)) {
	  std::ostringstream iss;
	  iss << "Cannot uniformly sample non bounded translation degrees of ";
	  iss << "freedom at rank ";
	  iss << index + i;
	  throw std::runtime_error (iss.str ());
	}
	else {
	  result [index + i] = lowerBound (i) +
	    (upperBound (i) - lowerBound (i)) * rand ()/RAND_MAX;
	}
      }
    }

    template class TranslationJointConfig <1>;
    template class TranslationJointConfig <2>;
    template class TranslationJointConfig <3>;

    namespace rotationJointConfig {
      void UnBounded::interpolate (ConfigurationIn_t q1, ConfigurationIn_t q2,
				   const value_type& u, const size_type& index,
				   ConfigurationOut_t result)
      {
	// interpolate on the unit circle
	double c1 = q1 [index], s1 = q1 [index + 1];
	double c2 = q2 [index], s2 = q2 [index + 1];
	double cosTheta = c1*c2 + s1*s2;
	double sinTheta = c1*s2 - s1*c2;
	double theta = atan2 (sinTheta, cosTheta);
	assert (fabs (sin (theta) - sinTheta) < 1e-8);
	if (fabs (theta) > 1e-6 && fabs (theta) < M_PI - 1e-6) {
	  result.segment (index, 2) =
	    (sin ((1-u)*theta)/sinTheta) * q1.segment (index, 2) +
	    (sin (u*theta)/sinTheta) * q2.segment (index, 2);
	} else if (fabs (theta) < 1e-6) { // theta = 0
	  result.segment (index, 2) =
	    (1-u) * q1.segment (index, 2) + u * q2.segment (index, 2);
	} else { // theta = +-M_PI
	  double theta0 = atan2 (q1 [index + 1], q1 [index]);
	  result [index] = cos (theta0 + u * theta);
	  result [index + 1] = sin (theta0 + u * theta);
	}
      }

      value_type UnBounded::squaredDistance (ConfigurationIn_t q1,
				             ConfigurationIn_t q2,
				             const size_type& index) const
      {
        const value_type d (distance (q1, q2, index));
	return d*d;
      }

      value_type UnBounded::distance (ConfigurationIn_t q1,
				      ConfigurationIn_t q2,
				      const size_type& index) const
      {
	Eigen::Matrix <value_type, 1, 1> result;
	difference (q1, q2, index, 0, result);
	return fabs (result [0]);
      }

      void UnBounded::integrate (ConfigurationIn_t q, vectorIn_t v,
				 const size_type& indexConfig,
				 const size_type& indexVelocity,
				 ConfigurationOut_t result) const
      {
	value_type omega = v [indexVelocity];
	value_type cosOmega = cos (omega);
	value_type sinOmega = sin (omega);
        value_type cosq = q [indexConfig];
        value_type sinq = q [indexConfig + 1];
	value_type norm2p = q.segment (indexConfig, 2).squaredNorm ();
	result [indexConfig    ] = (1.5-.5*norm2p) *
	  (cosOmega * cosq - sinOmega * sinq);
	result [indexConfig + 1] = (1.5-.5*norm2p) *
	  (sinOmega * cosq + cosOmega * sinq);
      }

      void UnBounded::difference (ConfigurationIn_t q1, ConfigurationIn_t q2,
				  const size_type& indexConfig,
				  const size_type& indexVelocity,
				  vectorOut_t result) const
      {
	value_type c1 = q1 [indexConfig];
	value_type s1 = q1 [indexConfig + 1];
	value_type c2 = q2 [indexConfig];
	value_type s2 = q2 [indexConfig + 1];

	// s1 = sin q1 c1 = cos (q1)
	// s2 = sin q2 c2 = cos (q2)
	// sin (q1 - q2) = s1*c2 - s2*c1
	// cos (q1 - q2) = c1*c2 + s1*s2
	result [indexVelocity] = atan2 (s1*c2 - s2*c1, c1*c2 + s1*s2);
      }

      bool UnBounded::isApprox (ConfigurationIn_t q1,
				ConfigurationIn_t q2,
				const size_type& indexConfig,
				const value_type& eps) const
      {
	return q1.segment <2> (indexConfig).isApprox
	  (q2.segment <2> (indexConfig));
      }

      /// Normalize configuration of joint
      void UnBounded::normalize (const size_type& index,
				 ConfigurationOut_t result) const
      {
	value_type c = result [index];
	value_type s = result [index + 1];
	value_type d = sqrt (c*c + s*s);
	result [index] /= d;
	result [index + 1] /= d;
      }

      void UnBounded::uniformlySample (const size_type& index,
				       ConfigurationOut_t result) const
      {
	value_type angle = -M_PI + 2* M_PI * rand ()/RAND_MAX;
	result [index] = cos (angle);
	result [index + 1] = sin (angle);
      }

      UnBounded::UnBounded () : JointConfiguration (2)
      {
      }

      void Bounded::interpolate (ConfigurationIn_t q1, ConfigurationIn_t q2,
				 const value_type& u, const size_type& index,
				 ConfigurationOut_t result)
      {
	// linearly interpolate
	result [index] = (1-u) * q1 [index] + u * q2 [index];
      }

      value_type Bounded::squaredDistance (ConfigurationIn_t q1,
                                           ConfigurationIn_t q2,
                                           const size_type& index) const
      {
	// linearly interpolate
        return (q2.segment<1>(index) - q1.segment<1>(index)).squaredNorm ();
      }

      value_type Bounded::distance (ConfigurationIn_t q1, ConfigurationIn_t q2,
				    const size_type& index) const
      {
	// linearly interpolate
	return fabs (q2 [index] - q1 [index]);
      }

      void Bounded::integrate (ConfigurationIn_t q, vectorIn_t v,
			       const size_type& indexConfig,
			       const size_type& indexVelocity,
			       ConfigurationOut_t result) const
      {
	value_type omega = v [indexVelocity];
	result [indexConfig] = q [indexConfig] + omega;
	if (result [indexConfig] < lowerBound (0)) {
	  result [indexConfig] = lowerBound (0);
	} else if (result [indexConfig] > upperBound (0)) {
	  result [indexConfig] = upperBound (0);
	}
      }

      void Bounded::difference (ConfigurationIn_t q1, ConfigurationIn_t q2,
				const size_type& indexConfig,
				const size_type& indexVelocity,
				vectorOut_t result) const
      {
	  result [indexVelocity] = q1 [indexConfig] - q2 [indexConfig];
      }

      /// Test that two configurations are close
      ///
      /// \param q1 first configuration,
      /// \param q2 second configuration,
      /// \param indexConfig index of first component of q corresponding to
      ///        the joint.
      /// \param eps numerical threshold
      /// \return true if the configurations are closer than the numerical
      /// threshold
      bool Bounded::isApprox (ConfigurationIn_t q1,
			      ConfigurationIn_t q2,
			      const size_type& indexConfig,
			      const value_type& eps) const
      {
	return (fabs (q1 [indexConfig] - q2 [indexConfig]) < eps);
      }

      /// Normalize configuration of joint
      /// Do nothing.
      void Bounded::normalize (const size_type&,
			       ConfigurationOut_t) const
      {
      }

      void Bounded::uniformlySample (const size_type& index,
				     ConfigurationOut_t result) const
      {
	result [index] = lowerBound (0) +
	  (upperBound (0) - lowerBound (0)) * rand ()/RAND_MAX;
      }

      Bounded::Bounded () : JointConfiguration (1)
      {
	isBounded (0, true);
	lowerBound (0, -M_PI);
	upperBound (0,  M_PI);
      }
    } // namespace rotationJointConfig

  } // namespace model
} // namespace hpp
