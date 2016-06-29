//
// Copyright (c) 2014 CNRS
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

#ifndef HPP_MODEL_JOINT_CONFIGURATION_HH
# define HPP_MODEL_JOINT_CONFIGURATION_HH

# include <cstddef>
# include <hpp/model/config.hh>
# include <hpp/model/fwd.hh>

namespace hpp {
  namespace model {
    /// Configuration of a Joint
    ///
    /// Depending on joint type, the configuration of the joint belongs to
    /// Lie groups with different structures.
    /// This abstract class sspecifies some methods that are specific to the
    /// joint configuration space, mainly
    /// \li straight interpolation
    /// \li distance between configurations
    /// \li sampling methods
    class HPP_MODEL_DLLAPI JointConfiguration {
    public:
      /// Constructor
      /// \param configSize dimension of the joint configuration size: used to
      /// resize the vector of bounds.
      JointConfiguration (size_type configSize);

      /// Destructor
      virtual ~JointConfiguration ();

      /// Interpolate two configurations of the joint
      /// \param q1, q2, two configurations to interpolate
      /// \param u in [0,1] position along the interpolation: q1 for u=0,
      /// q2 for u=1
      /// \param index index of first component of q1 and q2 corresponding to
      /// the joint.
      /// \retval result write joint configuration in
      /// result [index:index+nb dofs]
      ///
      /// q1 and q2 are configurations of the robot where coordinates between
      /// index and index + number of dofs - 1 correspond to the configuration
      /// of the joint:
      ///   \li a real value for translation joint and bounded rotation joints,
      ///   \li an angle for unbounded rotation joints,
      ///   \li x, y, z, roll, pitch, yaw for freeflyer joints.
      virtual void interpolate (ConfigurationIn_t q1,
				ConfigurationIn_t q2,
				const value_type& u,
				const size_type& index,
				ConfigurationOut_t result) = 0;

      /// Squared distance between two configurations of the joint
      /// \param q1, q2 two configurations of the robot
      /// \param index index of first component of q1 and q2 corresponding to
      /// the joint.
      virtual value_type squaredDistance (ConfigurationIn_t q1,
				          ConfigurationIn_t q2,
				          const size_type& index) const = 0;

      /// Distance between two configurations of the joint
      /// \param q1, q2 two configurations of the robot
      /// \param index index of first component of q1 and q2 corresponding to
      /// the joint.
      virtual value_type distance (ConfigurationIn_t q1,
				   ConfigurationIn_t q2,
				   const size_type& index) const = 0;

      /// Integrate constant derivative during unit time
      /// \param q initial configuration
      /// \param v joint velocity
      /// \param indexConfig index of first component of q corresponding to
      ///        the joint.
      /// \param indexVelocity index of first component of v corresponding to
      ///        the joint
      /// \retval result write joint configuration in
      /// result [indexConfig:indexConfig + joint config size]
      /// \note if result is beying bounds, return active bound.
      virtual void integrate (ConfigurationIn_t q,
			      vectorIn_t v,
			      const size_type& indexConfig,
			      const size_type& indexVelocity,
			      ConfigurationOut_t result) const = 0;

      /// Difference between two configurations
      ///
      /// \param q1 configuration,
      /// \param q2 configuration,
      /// \param indexConfig index of first component of q corresponding to
      ///        the joint.
      /// \param indexVelocity index of first component of v corresponding to
      ///        the joint
      /// \retval result[indexVelocity:indexVelocity+nbdofs] part of vector
      ///          representing the difference between q1 and q2.
      ///
      /// See derived classes for details
      virtual void difference (ConfigurationIn_t q1,
			       ConfigurationIn_t q2,
			       const size_type& indexConfig,
			       const size_type& indexVelocity,
			       vectorOut_t result) const = 0;

      /// Test that two configurations are close
      ///
      /// \param q1 first configuration,
      /// \param q2 second configuration,
      /// \param indexConfig index of first component of q corresponding to
      ///        the joint.
      /// \param eps numerical threshold
      /// \return true if the configurations are closer than the numerical
      /// threshold
      virtual bool isApprox (ConfigurationIn_t q1, ConfigurationIn_t q2,
			     const size_type& indexConfig,
			     const value_type& eps) const = 0;

      /// Normalize configuration of joint
      virtual void normalize (const size_type& index,
			      ConfigurationOut_t result) const = 0;
      /// Uniformly sample the configuration space of the joint
      /// \param index index of first component of q corresponding to the joint.
      /// \retval result write joint configuration in
      /// result [index:index+nb dofs]
      virtual void uniformlySample (const size_type& index,
				    ConfigurationOut_t result) const = 0;

      /// \name Bounds
      /// @{
      /// Set whether given degree of freedom is bounded
      void isBounded (size_type rank, bool bounded);
      /// Get whether given degree of freedom is bounded
      bool isBounded (size_type rank) const;
      /// Get lower bound of given degree of freedom
      value_type lowerBound (size_type rank) const;
      /// Get upper bound of given degree of freedom
      value_type upperBound (size_type rank) const;
      /// Set lower bound of given degree of freedom
      void lowerBound (size_type rank, value_type lowerBound);
      /// Set upper bound of given degree of freedom
      void upperBound (size_type rank, value_type upperBound);
      /// @}

    private:
      std::vector <bool> bounded_;
      vector_t lowerBounds_;
      vector_t upperBounds_;
    }; // class JointConfiguration

    /// Configuration of a JointAnchor
    class HPP_MODEL_DLLAPI AnchorJointConfig : public JointConfiguration
    {
    public:
      AnchorJointConfig ();
      virtual ~AnchorJointConfig ();
      virtual void interpolate (ConfigurationIn_t,
				ConfigurationIn_t,
				const value_type&,
				const size_type&,
				ConfigurationOut_t);

      /// Distance between two configurations of the joint
      /// \param q1, q2 two configurations of the robot
      /// \param index index of first component of q1 and q2 corresponding to
      /// the joint.
      /// \return 0
      virtual value_type distance (ConfigurationIn_t q1,
				   ConfigurationIn_t q2,
				   const size_type& index) const;
      value_type squaredDistance (ConfigurationIn_t q1,
				  ConfigurationIn_t q2,
				  const size_type& index) const;

      virtual void integrate (ConfigurationIn_t q,
			      vectorIn_t v,
			      const size_type& indexConfig,
			      const size_type& indexVelocity,
			      ConfigurationOut_t result) const;
      virtual void difference (ConfigurationIn_t q1,
			       ConfigurationIn_t q2,
			       const size_type& indexConfig,
			       const size_type& indexVelocity,
			       vectorOut_t result) const;
      /// \param q2 second configuration,
      /// \param indexConfig index of first component of q corresponding to
      ///        the joint.
      /// \param eps numerical threshold
      /// \return true if the configurations are closer than the numerical
      /// threshold
      virtual bool isApprox (ConfigurationIn_t q1, ConfigurationIn_t q2,
			     const size_type& indexConfig,
			     const value_type& eps) const;
      /// Normalize configuration of joint
      virtual void normalize (const size_type& index,
			      ConfigurationOut_t result) const;
      virtual void uniformlySample (const size_type& index,
				    ConfigurationOut_t result) const;
    }; // class AnchorJointConfig

    /// Configuration of a JointSO3
    class HPP_MODEL_DLLAPI SO3JointConfig : public JointConfiguration
    {
    public:
      SO3JointConfig ();
      virtual ~SO3JointConfig ();
      virtual void interpolate (ConfigurationIn_t q1,
				ConfigurationIn_t q2,
				const value_type& u,
				const size_type& index,
				ConfigurationOut_t result);

      /// Distance between two configurations of the joint
      /// \param q1, q2 two configurations of the robot
      /// \param index index of first component of q1 and q2 corresponding to
      /// the joint.
      /// \return the angle between the joint orientations
      virtual value_type distance (ConfigurationIn_t q1,
				   ConfigurationIn_t q2,
				   const size_type& index) const;
      value_type squaredDistance (ConfigurationIn_t q1,
				  ConfigurationIn_t q2,
				  const size_type& index) const;
      virtual void integrate (ConfigurationIn_t q,
			      vectorIn_t v,
			      const size_type& indexConfig,
			      const size_type& indexVelocity,
			      ConfigurationOut_t result) const;
      /// Difference between two configurations
      ///
      /// \param q1 configuration,
      /// \param q2 configuration,
      /// \param indexConfig index of first component of q corresponding to
      ///        the joint.
      /// \param indexVelocity index of first component of v corresponding to
      ///        the joint
      /// \retval result[index:index+ joint number dof] part of vector
      ///         representing the difference between q1 and q2.
      ///
      /// The difference is computed as follows:
      /// \f[
      /// \textbf{q}_1 [\texttt{index}:\texttt{index}+4] =
      /// \exp \left(\texttt{result}[\texttt{index}:\texttt{index}+3]_{\times}
      /// \right)\textbf{q}_2 [\texttt{index}:\texttt{index}+4]
      /// \f]
      virtual void difference (ConfigurationIn_t q1,
			       ConfigurationIn_t q2,
			       const size_type& indexConfig,
			       const size_type& indexVelocity,
			       vectorOut_t result) const;
      /// \param q2 second configuration,
      /// \param indexConfig index of first component of q corresponding to
      ///        the joint.
      /// \param eps numerical threshold
      /// \return true if the configurations are closer than the numerical
      /// threshold
      virtual bool isApprox (ConfigurationIn_t q1, ConfigurationIn_t q2,
			     const size_type& indexConfig,
			     const value_type& eps) const;
      /// Normalize configuration of joint
      virtual void normalize (const size_type& index,
			      ConfigurationOut_t result) const;
      virtual void uniformlySample (const size_type& index,
				    ConfigurationOut_t result) const;
    }; // class SO3JointConfig

    /// Configuration of a JointRotation
    class HPP_MODEL_DLLAPI RotationJointConfig : public JointConfiguration
    {
    public:
      /// Constructor
      /// \param configSize dimension of the joint configuration size: used to
      /// resize the vector of bounds.
      RotationJointConfig (size_type configSize);
      virtual ~RotationJointConfig ();
      virtual void interpolate (ConfigurationIn_t q1,
				ConfigurationIn_t q2,
				const value_type& u,
				const size_type& index,
				ConfigurationOut_t result) = 0;

      /// Distance between two configurations of the joint
      /// \param q1, q2 two configurations of the robot
      /// \param index index of first component of q1 and q2 corresponding to
      /// the joint.
      /// \return the angle between the joint orientations
      virtual value_type distance (ConfigurationIn_t q1,
				   ConfigurationIn_t q2,
				   const size_type& index) const = 0;
      virtual void integrate (ConfigurationIn_t q,
			      vectorIn_t v,
			      const size_type& indexConfig,
			      const size_type& indexVelocity,
			      ConfigurationOut_t result) const = 0;

      /// Difference between two configurations
      ///
      /// \param q1 configuration,
      /// \param q2 configuration,
      /// \param indexConfig index of first component of q corresponding to
      ///        the joint.
      /// \param indexVelocity index of first component of v corresponding to
      ///        the joint
      /// \retval result[indexVelocity] component of vector representing the
      /// difference between q1 and q2.
      ///
      /// If joint is bounded:
      /// \f[
      /// \texttt{result}[\texttt{index}] =
      /// \textbf{q}_1[\texttt{index}] - \textbf{q}_2[\texttt{index}]
      /// \f]
      ///
      /// If joint is not bounded:
      /// \f[
      /// \texttt{result}[\texttt{index}] =
      /// \textbf{q}_1[\texttt{index}] - \textbf{q}_2[\texttt{index}] + 2k\pi
      /// \f]
      /// where \f$k\f$ is such that \f$\texttt{result}[\texttt{index}]\f$ lies
      /// between \f$-\pi\f$ and \f$\pi\f$.
      virtual void difference (ConfigurationIn_t q1,
			       ConfigurationIn_t q2,
			       const size_type& indexConfig,
			       const size_type& indexVelocity,
			       vectorOut_t result) const = 0;

      /// \param q2 second configuration,
      /// \param indexConfig index of first component of q corresponding to
      ///        the joint.
      /// \param eps numerical threshold
      /// \return true if the configurations are closer than the numerical
      /// threshold
      virtual bool isApprox (ConfigurationIn_t q1, ConfigurationIn_t q2,
			     const size_type& indexConfig,
			     const value_type& eps) const;
      /// Normalize configuration of joint
      virtual void normalize (const size_type& index,
			      ConfigurationOut_t result) const;
      virtual void uniformlySample (const size_type& index,
				    ConfigurationOut_t result) const = 0;
    }; // class RotationJointConfig

    namespace rotationJointConfig {
      class HPP_MODEL_DLLAPI UnBounded : public JointConfiguration
      {
      public:
	UnBounded ();
	virtual ~UnBounded ()
	{
	}
	void interpolate (ConfigurationIn_t q1, ConfigurationIn_t q2,
			  const value_type& u, const size_type& index,
			  ConfigurationOut_t result);
	value_type distance (ConfigurationIn_t q1, ConfigurationIn_t q2,
			     const size_type& index) const;
        value_type squaredDistance (ConfigurationIn_t q1,
				    ConfigurationIn_t q2,
				    const size_type& index) const;
	void integrate (ConfigurationIn_t q, vectorIn_t v,
			const size_type& indexConfig,
			const size_type& indexVelocity,
			ConfigurationOut_t result) const;
	void difference (ConfigurationIn_t q1, ConfigurationIn_t q2,
			 const size_type& indexConfig,
			 const size_type& indexVelocity,
			 vectorOut_t result) const;
	/// \param q2 second configuration,
	/// \param indexConfig index of first component of q corresponding to
	///        the joint.
	/// \param eps numerical threshold
	/// \return true if the configurations are closer than the numerical
	/// threshold
	virtual bool isApprox (ConfigurationIn_t q1, ConfigurationIn_t q2,
			       const size_type& indexConfig,
			       const value_type& eps) const;
	/// Normalize configuration of joint
	virtual void normalize (const size_type& index,
			      ConfigurationOut_t result) const;
	void uniformlySample (const size_type& index,
			      ConfigurationOut_t result) const;
      }; // class UnBounded

      class HPP_MODEL_DLLAPI Bounded : public JointConfiguration
      {
      public:
	Bounded ();
	virtual ~Bounded ()
	{
	}
	void interpolate (ConfigurationIn_t q1, ConfigurationIn_t q2,
			  const value_type& u, const size_type& index,
			  ConfigurationOut_t result);
	value_type distance (ConfigurationIn_t q1, ConfigurationIn_t q2,
			     const size_type& index) const;
        value_type squaredDistance (ConfigurationIn_t q1,
				    ConfigurationIn_t q2,
				    const size_type& index) const;
	void integrate (ConfigurationIn_t q, vectorIn_t v,
			const size_type& indexConfig,
			const size_type& indexVelocity,
			ConfigurationOut_t result) const;
	void difference (ConfigurationIn_t q1, ConfigurationIn_t q2,
			 const size_type& indexConfig,
			 const size_type& indexVelocity,
			 vectorOut_t result) const;
	/// \param q2 second configuration,
	/// \param indexConfig index of first component of q corresponding to
	///        the joint.
	/// \param eps numerical threshold
	/// \return true if the configurations are closer than the numerical
	/// threshold
	virtual bool isApprox (ConfigurationIn_t q1, ConfigurationIn_t q2,
			       const size_type& indexConfig,
			       const value_type& eps) const;
	/// Normalize configuration of joint
	virtual void normalize (const size_type& index,
				ConfigurationOut_t result) const;
	void uniformlySample (const size_type& index,
			      ConfigurationOut_t result) const;
      }; // class Bounded
    } // namespace rotationJointConfig

    /// Configuration of a JointTranslation
    template <size_type dimension>
    class HPP_MODEL_DLLAPI TranslationJointConfig : public JointConfiguration
    {
    public:
      TranslationJointConfig ();
      virtual ~TranslationJointConfig ();
      virtual void interpolate (ConfigurationIn_t q1,
				ConfigurationIn_t q2,
				const value_type& u,
				const size_type& index,
				ConfigurationOut_t result);

      /// Distance between two configurations of the joint
      /// \param q1, q2 two configurations of the robot
      /// \param index index of first component of q1 and q2 corresponding to
      /// the joint.
      /// \return the absolute value of the joint value difference.
      virtual value_type distance (ConfigurationIn_t q1,
				   ConfigurationIn_t q2,
				   const size_type& index) const;
      value_type squaredDistance (ConfigurationIn_t q1,
				  ConfigurationIn_t q2,
				  const size_type& index) const;

      virtual void integrate (ConfigurationIn_t q,
			      vectorIn_t v,
			      const size_type& indexConfig,
			      const size_type& indexVelocity,
			      ConfigurationOut_t result) const;
      /// Difference between two configurations
      ///
      /// \param q1 configuration,
      /// \param q2 configuration,
      /// \param indexConfig index of first component of q corresponding to
      ///        the joint.
      /// \param indexVelocity index of first component of v corresponding to
      ///        the joint
      /// \retval result[index] component of vector representing the
      /// difference between q1 and q2.
      ///
      /// \f[
      /// \texttt{result}[\texttt{index}] =
      /// \textbf{q}_1[\texttt{index}] - \textbf{q}_2[\texttt{index}]
      /// \f]
      virtual void difference (ConfigurationIn_t q1,
			       ConfigurationIn_t q2,
			       const size_type& indexConfig,
			       const size_type& indexVelocity,
			       vectorOut_t result) const;

      /// \param q2 second configuration,
      /// \param indexConfig index of first component of q corresponding to
      ///        the joint.
      /// \param eps numerical threshold
      /// \return true if the configurations are closer than the numerical
      /// threshold
      virtual bool isApprox (ConfigurationIn_t q1, ConfigurationIn_t q2,
			     const size_type& indexConfig,
			     const value_type& eps) const;
      /// Normalize configuration of joint
      virtual void normalize (const size_type& index,
			      ConfigurationOut_t result) const;
      virtual void uniformlySample (const size_type& index,
				    ConfigurationOut_t result) const;
    }; // class TranslationJointConfig

  } // namespace model
} // namespace hpp

#endif // HPP_MODEL_JOINT_CONFIGURATION_HH
