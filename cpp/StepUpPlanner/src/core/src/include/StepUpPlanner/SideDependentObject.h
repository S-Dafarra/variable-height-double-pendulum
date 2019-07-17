#ifndef STEPUPPLANNER_SIDEDEPENDENTOBJECT_H
#define STEPUPPLANNER_SIDEDEPENDENTOBJECT_H

namespace StepUpPlanner {
    template<typename Object>
    class SideDependentObject;
}

template <typename Object>
class StepUpPlanner::SideDependentObject {
public:
    Object left;
    Object right;

    SideDependentObject() { }

    SideDependentObject(const Object& newLeft, const Object& newRight)
        : left(newLeft)
          ,right(newRight)
    { }

    void operator=(const SideDependentObject<Object>& other) {
        left = other.left;
        right = other.right;
    }

    ~SideDependentObject() { }
};

#endif // STEPUPPLANNER_SIDEDEPENDENTOBJECT_H
