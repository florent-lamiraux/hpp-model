///
/// Copyright (c) 2013, 2014 CNRS
/// Author: Florent Lamiraux
///
///
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

#include <fcl/math/vec_3f.h>
#include <hpp/util/debug.hh>
#include <hpp/model/body.hh>
#include <hpp/model/collision-object.hh>
#include <hpp/model/device.hh>
#include <hpp/model/fcl-to-eigen.hh>
#include <hpp/model/joint.hh>
#include <hpp/model/joint-configuration.hh>

#include "children-iterator.hh"

namespace hpp {
  namespace model {

    Joint::Joint (const Transform3f& initialPosition,
		  std::size_t configSize, std::size_t numberDof) :
      configuration_ (0x0), currentTransformation_ (initialPosition),
      positionInParentFrame_ (), T3f_ (), mass_ (0), massCom_ (),
      configSize_ (configSize), numberDof_ (numberDof),
      initialPosition_ (initialPosition),
      body_ (0x0), robot_ (0x0),
      name_ (), children_ (), parent_ (0x0), rankInConfiguration_ (-1),
      jacobian_ (), rankInParent_ (0)
    {
      positionInParentFrame_.setIdentity ();
      T3f_.setIdentity ();
      massCom_.setValue (0);
    }

    Joint::~Joint ()
    {
      delete body_;
    }

    const Transform3f& Joint::initialPosition () const
    {
      return initialPosition_;
    }

    const Transform3f& Joint::currentTransformation () const
    {
      return currentTransformation_;
    }

    void Joint::addChildJoint (Joint* joint)
    {
      if (!robot_) {
	throw std::runtime_error
	  ("Cannot insert child joint to a joint not belonging to a device.");
      }
      joint->robot_ = robot_;
      robot_->registerJoint (joint);
      joint->rankInParent_ = children_.size ();
      children_.push_back (joint);
      joint->parent_ = this;
      // Mjoint/parent = Mparent^{-1} Mjoint
      fcl::Transform3f Mp = currentTransformation_;
      fcl::Transform3f Mj = joint->currentTransformation_;
      joint->positionInParentFrame_ = Mp.inverse () * Mj;
    }

    void Joint::isBounded (std::size_t rank, bool bounded)
    {
      configuration_->isBounded (rank, bounded);
    }

    bool Joint::isBounded (std::size_t rank) const
    {
      return configuration_->isBounded (rank);
    }

    double Joint::lowerBound (std::size_t rank) const
    {
      return configuration_->lowerBound (rank);
    }

    double Joint::upperBound (std::size_t rank) const
    {
      return configuration_->upperBound (rank);
    }

    void Joint::lowerBound (std::size_t rank, double lower)
    {
      configuration_->lowerBound (rank, lower);
    }

    void Joint::upperBound (std::size_t rank, double upper)
    {
      configuration_->upperBound (rank, upper);
    }

    Body* Joint::linkedBody () const
    {
      return body_;
    }

    void Joint::setLinkedBody (Body* body) {
      body_ = body;
      body->joint (this);
      if (robot_) {
	robot_->computeMass ();
      }
    }

    void Joint::computePosition (ConfigurationIn_t configuration,
				 const Transform3f& parentConfig)
    {
      computeMotion (configuration, parentConfig);
      hppDout (info, "Joint " << name_);
      hppDout (info, "Joint position " << currentTransformation_);
      for (std::vector <JointPtr_t>::iterator itJoint = children_.begin ();
	   itJoint != children_.end (); itJoint++) {
	(*itJoint)->computePosition (configuration, currentTransformation_);
      }
    }

    void Joint::computeJacobian ()
    {
      for (ChildrenIterator it1 (this); !it1.end (); ++it1) {
	for (ChildrenIterator it2 (*it1); !it2.end (); ++it2) {
	  (*it1)->writeSubJacobian (*it2);
	}
      }
    }

    double Joint::computeMass ()
    {
      mass_ = 0;
      if (body_) {
	mass_ = body_->mass ();
      }
      for (std::vector <JointPtr_t>::iterator it =  children_.begin ();
	   it != children_.end (); it++) {
	mass_ += (*it)->computeMass ();
      }
      return mass_;
    }

    void Joint::computeMassTimesCenterOfMass ()
    {
      massCom_.setValue (0);
      if (body_) {
	massCom_ = currentTransformation_.transform
	  (body_->localCenterOfMass ()) * body_->mass ();
      }
      for (std::vector <JointPtr_t>::iterator it =  children_.begin ();
	   it != children_.end (); it++) {
	(*it)->computeMassTimesCenterOfMass ();
	massCom_ += (*it)->massCom_;
      }
    }

    JointAnchor::JointAnchor (const Transform3f& initialPosition) :
      Joint (initialPosition, 0, 0)
    {
      configuration_ = new AnchorJointConfig;
    }

    JointAnchor::~JointAnchor ()
    {
      delete configuration_;
    }

    void JointAnchor::computeMotion (ConfigurationIn_t,
				     const Transform3f& parentConfig)
    {
      currentTransformation_ = parentConfig * positionInParentFrame_;
    }

    void JointAnchor::writeSubJacobian (const JointPtr_t&)
    {
    }

    void JointAnchor::writeComSubjacobian (ComJacobian_t&,
					   const double&)
    {
    }

    JointSO3::JointSO3 (const Transform3f& initialPosition) :
      Joint (initialPosition, 4, 3)
    {
      configuration_ = new SO3JointConfig;
    }

    JointSO3::~JointSO3 ()
    {
      delete configuration_;
    }

    void JointSO3::computeMotion (ConfigurationIn_t configuration,
				  const Transform3f& parentConfig)
    {
      fcl::Quaternion3f p (configuration [rankInConfiguration ()],
			   configuration [rankInConfiguration () + 1],
			   configuration [rankInConfiguration () + 2],
			   configuration [rankInConfiguration () + 3]);
      T3f_.setQuatRotation (p);
      hppDout (info, "parentConfig = " << parentConfig);
      hppDout (info, "positionInParentFrame_ = " << positionInParentFrame_);
      hppDout (info, "T3f_ = " << T3f_);
      currentTransformation_ = parentConfig * positionInParentFrame_ * T3f_;
      hppDout (info, "currentTransformation_ = " << currentTransformation_);
    }
    static void cross (const fcl::Vec3f& x, JointJacobian_t& J, size_type row,
		       size_type col)
    {
      J (row + 0, col + 1) = -x [2]; J (row + 1, col + 0) = x [2];
      J (row + 0, col + 2) = x [1]; J (row + 2, col + 0) = -x [1];
      J (row + 1, col + 2) = -x [0]; J (row + 2, col + 1) = x [0];
    }

    static void cross (const fcl::Vec3f& x, ComJacobian_t& J, size_type row,
		       size_type col)
    {
      J (row + 0, col + 1) = -x [2]; J (row + 1, col + 0) = x [2];
      J (row + 0, col + 2) = x [1]; J (row + 2, col + 0) = -x [1];
      J (row + 1, col + 2) = -x [0]; J (row + 2, col + 1) = x [0];
    }

    void JointSO3::writeSubJacobian (const JointPtr_t& child)
    {
      std::size_t col = rankInVelocity ();
      // Set diagonal terms to 1
      child->jacobian () (3,col) = child->jacobian () (4,col+1) =
	child->jacobian () (5,col+2) = 1;
      hppDout (info, "currentTransformation_.getTranslation ()" <<
	       currentTransformation_.getTranslation ());
      hppDout (info, "child->currentTransformation ().getTranslation ()" <<
	       child->currentTransformation ().getTranslation ());
      cross (currentTransformation_.getTranslation () -
	     child->currentTransformation ().getTranslation (),
	     child->jacobian (), 0, col);
    }

    void JointSO3::writeComSubjacobian (ComJacobian_t& jacobian,
					const double& totalMass)
    {
      const fcl::Vec3f& center (currentTransformation_.getTranslation ());
      com_ = massCom_ * (1/mass_);
      size_type col = rankInVelocity ();
      cross ((mass_/totalMass)*(center-com_), jacobian, (size_type) 0, col);
    }

    JointRotation::JointRotation (const Transform3f& initialPosition) :
      Joint (initialPosition, 1, 1), R_ ()
    {
      configuration_ = new RotationJointConfig;
      R_.setIdentity ();
    }

    JointRotation::~JointRotation ()
    {
      delete configuration_;
    }

    void JointRotation::computeMotion (ConfigurationIn_t configuration,
				       const Transform3f& parentConfig)
    {
      angle_ = configuration [rankInConfiguration ()];
      R_ (1,1) = cos (angle_); R_ (1,2) = -sin (angle_);
      R_ (2,1) = sin (angle_); R_ (2,2) = cos (angle_);
      T3f_.setRotation (R_);
      hppDout (info, "parentConfig = " << parentConfig);
      hppDout (info, "positionInParentFrame_ = " << positionInParentFrame_);
      hppDout (info, "T3f_ = " << T3f_);
      currentTransformation_ = parentConfig * positionInParentFrame_ * T3f_;
      hppDout (info, "currentTransformation_ = " << currentTransformation_);
    }

    void JointRotation::writeSubJacobian (const JointPtr_t& child)
    {
      size_type col = rankInVelocity ();
      // Get rotation axis
      axis_ = currentTransformation_.getRotation ().getColumn (0);
      O2O1_ = currentTransformation_.getTranslation () -
	child->currentTransformation ().getTranslation ();
      cross_ = O2O1_.cross (axis_);
      child->jacobian () (0, col) = cross_ [0];
      child->jacobian () (1, col) = cross_ [1];
      child->jacobian () (2, col) = cross_ [2];
      child->jacobian () (3, col) = axis_ [0];
      child->jacobian () (4, col) = axis_ [1];
      child->jacobian () (5, col) = axis_ [2];
    }

    void JointRotation::writeComSubjacobian (ComJacobian_t& jacobian,
					     const double& totalMass)
    {
      size_type col = rankInVelocity ();
      axis_ = currentTransformation_.getRotation ().getColumn (0);
      com_ = massCom_ * (1/mass_);
      const fcl::Vec3f& center (currentTransformation_.getTranslation ());
      O2O1_ = center - com_;
      cross_ = (mass_/totalMass) * O2O1_.cross (axis_);
      jacobian (0, col) = cross_ [0];
      jacobian (1, col) = cross_ [1];
      jacobian (2, col) = cross_ [2];
    }

    JointTranslation::JointTranslation (const Transform3f& initialPosition) :
      Joint (initialPosition, 1, 1), t_ ()
    {
      configuration_ = new TranslationJointConfig;
      t_.setValue (0);
    }

    JointTranslation::~JointTranslation ()
    {
      delete configuration_;
    }

    void JointTranslation::computeMotion
    (ConfigurationIn_t configuration, const Transform3f& parentConfig)
    {
      t_ [0] = configuration [rankInConfiguration ()];
      T3f_.setTranslation (t_);
      currentTransformation_ = parentConfig * positionInParentFrame_ * T3f_;
    }

    void JointTranslation::writeSubJacobian (const JointPtr_t& child)
    {
      std::size_t col = rankInVelocity ();
      // Get translation axis
      axis_ = currentTransformation_.getRotation ().getColumn (0);
      child->jacobian () (0, col) = axis_ [0];
      child->jacobian () (1, col) = axis_ [1];
      child->jacobian () (2, col) = axis_ [2];
    }

    void JointTranslation::writeComSubjacobian (ComJacobian_t& jacobian,
						const double& totalMass)
    {
      std::size_t col = rankInVelocity ();
      // Get translation axis
      axis_ = currentTransformation_.getRotation ().getColumn (0);
      jacobian (0, col) = (mass_/totalMass) * axis_ [0];
      jacobian (1, col) = (mass_/totalMass) * axis_ [1];
      jacobian (2, col) = (mass_/totalMass) * axis_ [2];
    }

    std::ostream& Joint::display (std::ostream& os) const
    {
      os << "Joint: " << name () << std::endl;
      if (configSize () != 0)
	os << "Rank in configuration: "
	   << rankInConfiguration()
	   << std::endl;
      else
	os << "Anchor joint" << std::endl;
      os << "Current transformation:" << std::endl;
      os << currentTransformation() << std:: endl;
      os << std::endl;
      os << "children: ";
      for (hpp::model::JointVector_t::const_iterator it =
	     children_.begin (); it != children_.end (); it++) {
	os << " ," << (*it)->name ();
      }
      os << std::endl;
      hpp::model::Body* body = linkedBody();
      if (body) {
	os << "Attached body:" << std::endl;
	os << "Mass of the attached body: " << body->mass()
	   << std::endl;
	os << "Local center of mass:" << body->localCenterOfMass()
	   << std::endl;
	os << "Inertia matrix:" << std::endl;
	os << body->inertiaMatrix() << std::endl;
	os << "geometric objects" << std::endl;
	const hpp::model::ObjectVector_t& colObjects =
	  body->innerObjects (hpp::model::COLLISION);
	for (hpp::model::ObjectVector_t::const_iterator it =
	       colObjects.begin (); it != colObjects.end (); it++) {
	  os << "name: " << (*it)->name () << std::endl;
	  os << "position :" << std::endl;
	  const fcl::Transform3f& transform ((*it)->fcl ()->getTransform ());
	  os << transform;
	}
      } else {
	os << "No body" << std::endl;
      }
      for (unsigned int iChild=0; iChild < numberChildJoints ();
	   iChild++) {
	hpp::model::Joint* child = childJoint (iChild);
	os << *(child) << std::endl;
	os <<std::endl;
      }
      return os;
    }

  } // namespace model
} // namespace hpp

std::ostream& operator<< (std::ostream& os , const fcl::Transform3f& trans)
{
  const fcl::Matrix3f& R (trans.getRotation ());
  const fcl::Vec3f& t (trans.getTranslation ());
  const fcl::Quaternion3f& q (trans.getQuatRotation ());
  os << "rotation matrix: " << R << std::endl;
  os << "rotation quaternion: " << "(" << q.getW () << ", "
     << q.getX () << ", " << q.getY () << ", " << q.getZ () << ")"
     << std::endl;
  os << "translation: " << t << std::endl;
  return os;
}

std::ostream& operator<< (std::ostream& os, const hpp::model::Joint& joint)
{
  return joint.display (os);
}
