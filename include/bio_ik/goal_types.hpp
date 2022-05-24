/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2016-2017, Philipp Sebastian Ruppel
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the copyright holder nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

#pragma once

#include <bio_ik/goal.hpp>

#include <bio_ik/robot_info.hpp>

#include <tf2/LinearMath/Vector3.h>
#include <tf2/LinearMath/Quaternion.h>

#include <moveit/collision_detection/collision_common.h>
#include <moveit/collision_detection_fcl/collision_common.h>

#include <map>
#include <unordered_set>

#include <geometric_shapes/bodies.h>
#include <geometric_shapes/shapes.h>

namespace bio_ik
{

class LinkGoalBase : public Goal
{
    std::string link_name_;

public:
    LinkGoalBase()
    {
        weight_ = 1;
        link_name_ = "";
    }
    LinkGoalBase(const std::string& link_name, double weight)
    {
        weight_ = weight;
        link_name_ = link_name;
    }
    virtual void describe(GoalContext& context) const
    {
        Goal::describe(context);
        context.addLink(link_name_);
    }
    void setLinkName(const std::string& link_name) { link_name_ = link_name; }
    const std::string& getLinkName() const { return link_name_; }
};

class PositionGoal : public LinkGoalBase
{
    tf2::Vector3 position_;

public:
    PositionGoal()
        : position_(0, 0, 0)
    {
    }
    PositionGoal(const std::string& link_name, const tf2::Vector3& position, double weight = 1.0)
        : LinkGoalBase(link_name, weight)
        , position_(position)
    {
    }
    inline const tf2::Vector3& getPosition() const { return position_; }
    inline void setPosition(const tf2::Vector3& position) { position_ = position; }
    virtual double evaluate(const GoalContext& context) const { return context.getLinkFrame().getPosition().distance2(getPosition()); }
};

class OrientationGoal : public LinkGoalBase
{
    tf2::Quaternion orientation_;

public:
    OrientationGoal()
        : orientation_(0, 0, 0, 1)
    {
    }
    OrientationGoal(const std::string& link_name, const tf2::Quaternion& orientation, double weight = 1.0)
        : LinkGoalBase(link_name, weight)
        , orientation_(orientation.normalized())
    {
    }
    inline const tf2::Quaternion& getOrientation() const { return orientation_; }
    inline void setOrientation(const tf2::Quaternion& orientation) { orientation_ = orientation.normalized(); }
    virtual double evaluate(const GoalContext& context) const
    {
        // return getOrientation().distance2(context.getLinkFrame().getOrientation());
        // return (getOrientation() - getOrientation().nearest(context.getLinkFrame().getOrientation())).length2();
        return fmin((getOrientation() - context.getLinkFrame().getOrientation()).length2(), (getOrientation() + context.getLinkFrame().getOrientation()).length2());
        /*return
            (getOrientation() - context.getLinkFrame().getOrientation()).length2() *
            (getOrientation() + context.getLinkFrame().getOrientation()).length2() * 0.5;*/
    }
};

class PoseGoal : public LinkGoalBase
{
    Frame frame_;
    double rotation_scale_;

public:
    PoseGoal()
        : frame_(Frame::identity())
        , rotation_scale_(0.5)
    {
    }
    PoseGoal(const std::string& link_name, const tf2::Vector3& position, const tf2::Quaternion& orientation, double weight = 1.0)
        : LinkGoalBase(link_name, weight)
        , frame_(position, orientation.normalized())
        , rotation_scale_(0.5)
    {
    }
    inline const tf2::Vector3& getPosition() const { return frame_.getPosition(); }
    inline void setPosition(const tf2::Vector3& position) { frame_.setPosition(position); }
    inline const tf2::Quaternion& getOrientation() const { return frame_.getOrientation(); }
    inline void setOrientation(const tf2::Quaternion& orientation) { frame_.setOrientation(orientation.normalized()); }
    inline double getRotationScale() const { return rotation_scale_; }
    inline void setRotationScale(double rotation_scale) { rotation_scale_ = rotation_scale; }
    virtual double evaluate(const GoalContext& context) const
    {
        double e = 0.0;
        e += context.getLinkFrame().getPosition().distance2(getPosition());

        /*e +=
            (getOrientation() - context.getLinkFrame().getOrientation()).length2() *
            (getOrientation() + context.getLinkFrame().getOrientation()).length2() *
            (rotation_scale_ * rotation_scale_) * 0.5;*/

        /*double a = getOrientation().angleShortestPath(context.getLinkFrame().getOrientation());
        e += a * a;
        return e;*/

        /*e += 1 - getOrientation().dot(context.getLinkFrame().getOrientation());
        return e;*/

        /*double l = getOrientation().length2() * context.getLinkFrame().getOrientation().length2();
        //double x = _mm_rsqrt_ss(_mm_set_ss((float)l))[0];
        double x = 1.0 / l;
        e += (1 - getOrientation().dot(context.getLinkFrame().getOrientation()) * x) * (rotation_scale_ * rotation_scale_);
        return e;*/

        e += fmin((getOrientation() - context.getLinkFrame().getOrientation()).length2(), (getOrientation() + context.getLinkFrame().getOrientation()).length2()) * (rotation_scale_ * rotation_scale_);

        // e += (1.0 - getOrientation().dot(context.getLinkFrame().getOrientation())) * (rotation_scale_ * rotation_scale_);

        // e += (getOrientation() - context.getLinkFrame().getOrientation()).length2() * (rotation_scale_ * rotation_scale_);
        // ROS_ERROR("r %f", (getOrientation() - context.getLinkFrame().getOrientation()).length2());
        // e += (getOrientation() - getOrientation().nearest(context.getLinkFrame().getOrientation())).length2() * (rotation_scale_ * rotation_scale_);
        return e;
    }
};

class LookAtGoal : public LinkGoalBase
{
    tf2::Vector3 axis_;
    tf2::Vector3 target_;

public:
    LookAtGoal()
        : axis_(1, 0, 0)
        , target_(0, 0, 0)
    {
    }
    LookAtGoal(const std::string& link_name, const tf2::Vector3& axis, const tf2::Vector3& target, double weight = 1.0)
        : LinkGoalBase(link_name, weight)
        , axis_(axis)
        , target_(target)
    {
    }
    const tf2::Vector3& getAxis() const { return axis_; }
    const tf2::Vector3& getTarget() const { return target_; }
    void setAxis(const tf2::Vector3& axis) { axis_ = axis.normalized(); }
    void setTarget(const tf2::Vector3& target) { target_ = target; }
    virtual double evaluate(const GoalContext& context) const
    {
        auto& fb = context.getLinkFrame();
        tf2::Vector3 axis;
        quat_mul_vec(fb.getOrientation(), axis_, axis);
        return (target_ - fb.getPosition()).normalized().distance2(axis.normalized());
        // return (target_ - axis * axis.dot(target_ - fb.getPosition())).distance2(fb.getPosition());
    }
};

class MaxDistanceGoal : public LinkGoalBase
{
    tf2::Vector3 target_;
    double distance_;

public:
    MaxDistanceGoal()
        : target_(0, 0, 0)
        , distance_(1)
    {
    }
    MaxDistanceGoal(const std::string& link_name, const tf2::Vector3& target, double distance, double weight = 1.0)
        : LinkGoalBase(link_name, weight)
        , target_(target)
        , distance_(distance)
    {
    }
    const tf2::Vector3& getTarget() const { return target_; }
    void setTarget(const tf2::Vector3& t) { target_ = t; }
    double getDistance() const { return distance_; }
    void setDistance(double d) { distance_ = d; }
    virtual double evaluate(const GoalContext& context) const
    {
        auto& fb = context.getLinkFrame();
        double d = fmax(0.0, fb.getPosition().distance(target_) - distance_);
        return d * d;
    }
};

class MinDistanceGoal : public LinkGoalBase
{
    tf2::Vector3 target_;
    double distance_;

public:
    MinDistanceGoal()
        : target_(0, 0, 0)
        , distance_(1)
    {
    }
    MinDistanceGoal(const std::string& link_name, const tf2::Vector3& target, double distance, double weight = 1.0)
        : LinkGoalBase(link_name, weight)
        , target_(target)
        , distance_(distance)
    {
    }
    const tf2::Vector3& getTarget() const { return target_; }
    void setTarget(const tf2::Vector3& t) { target_ = t; }
    double getDistance() const { return distance_; }
    void setDistance(double d) { distance_ = d; }
    virtual double evaluate(const GoalContext& context) const
    {
        auto& fb = context.getLinkFrame();
        double d = fmax(0.0, distance_ - fb.getPosition().distance(target_));
        return d * d;
    }
};

class LineGoal : public LinkGoalBase
{
    tf2::Vector3 position_;
    tf2::Vector3 direction_;

public:
    LineGoal()
        : position_(0, 0, 0)
        , direction_(0, 0, 0)
    {
    }
    LineGoal(const std::string& link_name, const tf2::Vector3& position, const tf2::Vector3& direction, double weight = 1.0)
        : LinkGoalBase(link_name, weight)
        , position_(position)
        , direction_(direction.normalized())
    {
    }
    const tf2::Vector3& getPosition() const { return position_; }
    void setPosition(const tf2::Vector3& p) { position_ = p; }
    const tf2::Vector3& getDirection() const { return direction_; }
    void setDirection(const tf2::Vector3& d) { direction_ = d.normalized(); }
    virtual double evaluate(const GoalContext& context) const
    {
        auto& fb = context.getLinkFrame();
        return position_.distance2(fb.getPosition() - direction_ * direction_.dot(fb.getPosition() - position_));
    }
};

#if (MOVEIT_FCL_VERSION < FCL_VERSION_CHECK(0, 6, 0))
class TouchGoal : public LinkGoalBase
{
    tf2::Vector3 position_;
    tf2::Vector3 normal_;
    struct CollisionShape
    {
        std::vector<Vector3> vertices;
        std::vector<fcl::Vec3f> points;
        std::vector<int> polygons;
        std::vector<fcl::Vec3f> plane_normals;
        std::vector<double> plane_dis;
        collision_detection::FCLGeometryConstPtr geometry;
        Frame frame;
        std::vector<std::vector<size_t>> edges;
    };
    struct CollisionLink
    {
        bool initialized;
        std::vector<std::shared_ptr<CollisionShape>> shapes;
        CollisionLink()
            : initialized(false)
        {
        }
    };
    struct CollisionModel
    {
        std::vector<CollisionLink> collision_links;
    };
    mutable CollisionModel* collision_model;
    mutable const moveit::core::LinkModel* link_model;

public:
    TouchGoal()
        : position_(0, 0, 0)
        , normal_(0, 0, 0)
    {
    }
    TouchGoal(const std::string& link_name, const tf2::Vector3& position, const tf2::Vector3& normal, double weight = 1.0)
        : LinkGoalBase(link_name, weight)
        , position_(position)
        , normal_(normal.normalized())
    {
    }
    virtual void describe(GoalContext& context) const;
    virtual double evaluate(const GoalContext& context) const;
};
#endif

class AvoidJointLimitsGoal : public Goal
{
public:
    AvoidJointLimitsGoal(double weight = 1.0, bool secondary = true)
    {
        weight_ = weight;
        secondary_ = secondary;
    }
    virtual double evaluate(const GoalContext& context) const
    {
        auto& info = context.getRobotInfo();
        double sum = 0.0;
        for(size_t i = 0; i < context.getProblemVariableCount(); i++)
        {
            size_t ivar = context.getProblemVariableIndex(i);
            if(info.getClipMax(ivar) == DBL_MAX) continue;
            double d = context.getProblemVariablePosition(i) - (info.getMin(ivar) + info.getMax(ivar)) * 0.5;
            d = fmax(0.0, fabs(d) * 2.0 - info.getSpan(ivar) * 0.5);
            d *= context.getProblemVariableWeight(i);
            sum += d * d;
        }
        return sum;
    }
};

class CenterJointsGoal : public Goal
{
public:
    CenterJointsGoal(double weight = 1.0, bool secondary = true)
    {
        weight_ = weight;
        secondary_ = secondary;
    }
    virtual double evaluate(const GoalContext& context) const
    {
        auto& info = context.getRobotInfo();
        double sum = 0.0;
        for(size_t i = 0; i < context.getProblemVariableCount(); i++)
        {
            size_t ivar = context.getProblemVariableIndex(i);
            if(info.getClipMax(ivar) == DBL_MAX) continue;
            double d = context.getProblemVariablePosition(i) - (info.getMin(ivar) + info.getMax(ivar)) * 0.5;
            d *= context.getProblemVariableWeight(i);
            sum += d * d;
        }
        return sum;
    }
};

class RegularizationGoal : public Goal
{
public:
  RegularizationGoal(double weight = 1.0)
  {
    weight_ = weight;
  }
  virtual double evaluate(const GoalContext &context) const
  {
    double sum = 0.0;
    for (size_t i = 0; i < context.getProblemVariableCount(); i++)
    {
      double d = context.getProblemVariablePosition(i) - context.getProblemVariableInitialGuess(i);
      sum += d * d;
    }
    return sum;
  }
};

class MinimalDisplacementGoal : public Goal
{
public:
    MinimalDisplacementGoal(double weight = 1.0, bool secondary = true)
    {
        weight_ = weight;
        secondary_ = secondary;
    }
    virtual double evaluate(const GoalContext& context) const
    {
        double sum = 0.0;
        for(size_t i = 0; i < context.getProblemVariableCount(); i++)
        {
            double d = context.getProblemVariablePosition(i) - context.getProblemVariableInitialGuess(i);
            d *= context.getProblemVariableWeight(i);
            sum += d * d;
        }
        return sum;
    }
};

class JointVariableGoal : public Goal
{
    std::string variable_name_;
    double variable_position_;

public:
    JointVariableGoal()
        : variable_position_(0)
    {
    }
    JointVariableGoal(const std::string& variable_name, double variable_position, double weight = 1.0, bool secondary = false)
        : variable_name_(variable_name)
        , variable_position_(variable_position)
    {
        weight_ = weight;
        secondary_ = secondary;
    }
    double getVariablePosition() const { return variable_position_; }
    void setVariablePosition(double p) { variable_position_ = p; }
    const std::string& getVariableName() const { return variable_name_; }
    void setVariableName(const std::string& n) { variable_name_ = n; }
    virtual void describe(GoalContext& context) const
    {
        Goal::describe(context);
        context.addVariable(variable_name_);
    }
    virtual double evaluate(const GoalContext& context) const
    {
        double d = variable_position_ - context.getVariablePosition();
        return d * d;
    }
};

class JointFunctionGoal : public Goal
{
    std::vector<std::string> variable_names_;
    std::function<void(std::vector<double>&)> function_;

public:
    JointFunctionGoal() {}
    JointFunctionGoal(const std::vector<std::string>& variable_names, const std::function<void(std::vector<double>&)>& function, double weight = 1.0, bool secondary = false)
        : variable_names_(variable_names)
        , function_(function)
    {
        weight_ = weight;
        secondary_ = secondary;
    }
    void setJointVariableNames(const std::vector<std::string>& n) { variable_names_ = n; }
    void setJointVariableFunction(const std::function<void(std::vector<double>&)>& f) { function_ = f; }
    virtual void describe(GoalContext& context) const
    {
        Goal::describe(context);
        for(auto& variable_name : variable_names_)
            context.addVariable(variable_name);
    }
    virtual double evaluate(const GoalContext& context) const
    {
        auto& temp_vector = context.getTempVector();
        temp_vector.resize(variable_names_.size());
        for(size_t i = 0; i < variable_names_.size(); i++)
            temp_vector[i] = context.getVariablePosition(i);
        function_(temp_vector);
        double sum = 0.0;
        for(size_t i = 0; i < variable_names_.size(); i++)
        {
            double d = temp_vector[i] - context.getVariablePosition(i);
            sum += d * d;
        }
        return sum;
    }
};

class BalanceGoal : public Goal
{
    tf2::Vector3 target_, axis_;
    struct BalanceInfo
    {
        tf2::Vector3 center;
        double weight;
    };
    mutable std::vector<BalanceInfo> balance_infos;

public:
    BalanceGoal()
        : target_(0, 0, 0)
        , axis_(0, 0, 1)
    {
    }
    BalanceGoal(const tf2::Vector3& target, double weight = 1.0)
        : target_(target)
        , axis_(0, 0, 1)
    {
        weight_ = weight;
    }
    const tf2::Vector3& getTarget() const { return target_; }
    const tf2::Vector3& getAxis() const { return axis_; }
    void setTarget(const tf2::Vector3& target) { target_ = target; }
    void setAxis(const tf2::Vector3& axis) { axis_ = axis; }
    virtual void describe(GoalContext& context) const;
    virtual double evaluate(const GoalContext& context) const;
};

class LinkFunctionGoal : public LinkGoalBase
{
    std::function<double(const tf2::Vector3&, const tf2::Quaternion&)> function_;

public:
    LinkFunctionGoal() {}
    LinkFunctionGoal(const std::string& link_name, const std::function<double(const tf2::Vector3&, const tf2::Quaternion&)>& function, double weight = 1.0)
        : LinkGoalBase(link_name, weight)
        , function_(function)
    {
    }
    void setLinkFunction(const std::function<double(const tf2::Vector3&, const tf2::Quaternion&)>& f) { function_ = f; }
    virtual double evaluate(const GoalContext& context) const { return function_(context.getLinkFrame().getPosition(), context.getLinkFrame().getOrientation()); }
};

class SideGoal : public LinkGoalBase
{
    tf2::Vector3 axis_;
    tf2::Vector3 direction_;

public:
    SideGoal()
        : axis_(0, 0, 1)
        , direction_(0, 0, 1)
    {
    }
    SideGoal(const std::string& link_name, const tf2::Vector3& axis, const tf2::Vector3& direction, double weight = 1.0)
        : LinkGoalBase(link_name, weight)
        , axis_(axis)
        , direction_(direction)
    {
    }
    const tf2::Vector3& getAxis() const { return axis_; }
    const tf2::Vector3& getDirection() const { return direction_; }
    void setAxis(const tf2::Vector3& a) { axis_ = a.normalized(); }
    void setDirection(const tf2::Vector3& d) { direction_ = d.normalized(); }
    virtual double evaluate(const GoalContext& context) const
    {
        auto& fb = context.getLinkFrame();
        Vector3 v;
        quat_mul_vec(fb.getOrientation(), axis_, v);
        double f = fmax(0.0, v.dot(direction_));
        return f * f;
    }
};

class DirectionGoal : public LinkGoalBase
{
    tf2::Vector3 axis_;
    tf2::Vector3 direction_;

public:
    DirectionGoal()
        : axis_(0, 0, 1)
        , direction_(0, 0, 1)
    {
    }
    DirectionGoal(const std::string& link_name, const tf2::Vector3& axis, const tf2::Vector3& direction, double weight = 1.0)
        : LinkGoalBase(link_name, weight)
        , axis_(axis)
        , direction_(direction)
    {
    }
    const tf2::Vector3& getAxis() const { return axis_; }
    const tf2::Vector3& getDirection() const { return direction_; }
    void setAxis(const tf2::Vector3& a) { axis_ = a.normalized(); }
    void setDirection(const tf2::Vector3& d) { direction_ = d.normalized(); }
    virtual double evaluate(const GoalContext& context) const
    {
        auto& fb = context.getLinkFrame();
        Vector3 v;
        quat_mul_vec(fb.getOrientation(), axis_, v);
        return v.distance2(direction_);
    }
};

class ConeGoal : public LinkGoalBase
{
    tf2::Vector3 position_;
    double position_weight_;
    tf2::Vector3 axis_;
    tf2::Vector3 direction_;
    double angle_;

public:
    const tf2::Vector3& getPosition() const { return position_; }
    double getPositionWeight() const { return position_weight_; }
    const tf2::Vector3& getAxis() const { return axis_; }
    const tf2::Vector3& getDirection() const { return direction_; }
    double getAngle() const { return angle_; }
    void setPosition(const tf2::Vector3& p) { position_ = p; }
    void setPositionWeight(double w) { position_weight_ = w; }
    void setAxis(const tf2::Vector3& a) { axis_ = a.normalized(); }
    void setDirection(const tf2::Vector3& d) { direction_ = d.normalized(); }
    void setAngle(double a) { angle_ = a; }
    ConeGoal()
        : position_(0, 0, 0)
        , position_weight_(0)
        , axis_(0, 0, 1)
        , direction_(0, 0, 1)
        , angle_(0)
    {
    }
    ConeGoal(const std::string& link_name, const tf2::Vector3& axis, const tf2::Vector3& direction, double angle, double weight = 1.0)
        : LinkGoalBase(link_name, weight)
        , position_(0, 0, 0)
        , position_weight_(0)
        , axis_(axis)
        , direction_(direction)
        , angle_(angle)
    {
    }
    ConeGoal(const std::string& link_name, const tf2::Vector3& position, const tf2::Vector3& axis, const tf2::Vector3& direction, double angle, double weight = 1.0)
        : LinkGoalBase(link_name, weight)
        , position_(position)
        , position_weight_(1)
        , axis_(axis)
        , direction_(direction)
        , angle_(angle)
    {
    }
    ConeGoal(const std::string& link_name, const tf2::Vector3& position, double position_weight, const tf2::Vector3& axis, const tf2::Vector3& direction, double angle, double weight = 1.0)
        : LinkGoalBase(link_name, weight)
        , position_(position)
        , position_weight_(position_weight)
        , axis_(axis)
        , direction_(direction)
        , angle_(angle)
    {
    }
    virtual double evaluate(const GoalContext& context) const
    {
        double sum = 0.0;
        auto& fb = context.getLinkFrame();
        Vector3 v;
        quat_mul_vec(fb.getOrientation(), axis_, v);
        double d = fmax(0.0, v.angle(direction_) - angle_);
        sum += d * d;
        double w = position_weight_;
        sum += w * w * (position_ - fb.getPosition()).length2();
        return sum;
    }
};
}
