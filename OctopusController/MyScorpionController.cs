using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using UnityEngine;


namespace OctopusController
{
  
    public class MyScorpionController
    {
        public struct PositionRotation
        {
            Vector3 position;
            Quaternion rotation;

            public PositionRotation(Vector3 position, Quaternion rotation)
            {
                this.position = position;
                this.rotation = rotation;
            }

            // PositionRotation to Vector3
            public static implicit operator Vector3(PositionRotation pr)
            {
                return pr.position;
            }
            // PositionRotation to Quaternion
            public static implicit operator Quaternion(PositionRotation pr)
            {
                return pr.rotation;
            }
        }

        //TAIL
        Transform tailTarget;
        Transform tailEndEffector;
        MyTentacleController _tail;
        float animationRange = 5f;

        float[] _tailBoneAngles;
        Vector3[] _tailBoneAxis;
        Vector3[] _tailBoneOffsets;

        public delegate float ErrorFunction(Vector3 target, float[] solution);
        private ErrorFunction _errorFunction;

        public float DeltaGradient = 0.1f; // Used to simulate gradient (degrees)
        public float LearningRate = 2.0f; // How much we move depending on the gradient

        public float StopThreshold = 0.1f; // If closer than this, it stops
        public float SlowdownThreshold = 0.25f; // If closer than this, it linearly slows down


        //LEGS
        Transform[] legTargets;
        Transform[] legFutureBases;
        MyTentacleController[] _legs = new MyTentacleController[6];

        
        #region public
        public void InitLegs(Transform[] LegRoots,Transform[] LegFutureBases, Transform[] LegTargets)
        {
            _legs = new MyTentacleController[LegRoots.Length];
            //Legs init
            for(int i = 0; i < LegRoots.Length; i++)
            {
                _legs[i] = new MyTentacleController();
                _legs[i].LoadTentacleJoints(LegRoots[i], TentacleMode.LEG);
                //TODO: initialize anything needed for the FABRIK implementation
            }

        }

        public void InitTail(Transform TailBase)
        {
            _tail = new MyTentacleController();
            _tail.LoadTentacleJoints(TailBase, TentacleMode.TAIL);

            //TODO: Initialize anything needed for the Gradient Descent implementation

            _tailBoneAxis = new Vector3[_tail.Bones.Length];
            _tailBoneAngles = new float[_tail.Bones.Length];
            _tailBoneOffsets = new Vector3[_tail.Bones.Length];
            for (int i = 0; i < _tail.Bones.Length; ++i)
            {
                _tail.Bones[i].localRotation.ToAngleAxis(out _tailBoneAngles[i], out _tailBoneAxis[i]);

                if (i > 0)
                {
                    _tailBoneOffsets[i] = Quaternion.Inverse(_tail.Bones[i-1].rotation) * (_tail.Bones[i].position - _tail.Bones[i - 1].position);
                }
                else
                {
                    _tailBoneOffsets[i] = _tail.Bones[i].position;
                }
            }
            _tailBoneAxis[0] = _tail.Bones[0].forward;


            _errorFunction = DistanceFromTarget;

            tailEndEffector = _tail.Bones[_tail.Bones.Length - 1];
        }

        //TODO: Check when to start the animation towards target and implement Gradient Descent method to move the joints.
        public void NotifyTailTarget(Transform target)
        {
            if (Vector3.Distance(target.position, tailEndEffector.position) < animationRange)
            {
                tailTarget = target;
            }
        }

        //TODO: Notifies the start of the walking animation
        public void NotifyStartWalk()
        {

        }

        //TODO: create the apropiate animations and update the IK from the legs and tail

        public void UpdateIK()
        {
            updateTail();
        }
        #endregion


        #region private
        //TODO: Implement the leg base animations and logic
        private void updateLegPos()
        {
            //check for the distance to the futureBase, then if it's too far away start moving the leg towards the future base position
            //
        }
        
        //TODO: implement fabrik method to move legs 
        private void updateLegs()
        {

        }

        //TODO: implement Gradient Descent method to move tail if necessary
        private void updateTail()
        {
            if (tailTarget != null)
            {
                if (Vector3.Distance(tailTarget.position, tailEndEffector.position) > StopThreshold)
                {
                    ApproachTarget(tailTarget.position);
                }                    
            }
        }

        #endregion



        // Gradient Descent functions
        public void ApproachTarget(Vector3 target)
        {
            //TODO (done)
            // Update rotations
            for (int i = 0; i < _tailBoneAngles.Length; ++i)
            {
                _tailBoneAngles[i] = _tailBoneAngles[i] - (LearningRate * CalculateGradient(target, _tailBoneAngles, i, DeltaGradient));
            }

            // TODO 
            // Aplly rotations
            for (int i = 0; i < _tailBoneAngles.Length; i++)
            {
                //_tail.Bones[i].localRotation = Quaternion.identity;
                _tail.Bones[i].localRotation = Quaternion.AngleAxis(_tailBoneAngles[i], _tailBoneAxis[i]);
            }
        }


        public float CalculateGradient(Vector3 target, float[] Solution, int i, float delta)
        {
            //TODO
            Solution[i] += delta; // Temporaraly get delta solution
            float deltaDistamceFromTarget = _errorFunction(target, Solution);

            Solution[i] -= delta; // Reset Solution

            return (deltaDistamceFromTarget - _errorFunction(target, Solution)) / delta;
        }

        // Returns the distance from the target, given a solution
        public float DistanceFromTarget(Vector3 target, float[] Solution)
        {
            Vector3 point = ForwardKinematics(Solution);
            return Vector3.Distance(point, target);
        }


        /* Simulates the forward kinematics,
         * given a solution. */


        public PositionRotation ForwardKinematics(float[] Solution)
        {
            Vector3 prevPoint = _tail.Bones[0].transform.position;

            // Takes object initial rotation into account
            //Quaternion rotation = transform.rotation;
            Quaternion rotation = Quaternion.identity; // ??? ask if correct

            //TODO (done)
            for (int i = 0; i < Solution.Length - 1; ++i)
            {
                Vector3 prev = prevPoint;
                rotation = rotation * Quaternion.AngleAxis(Solution[i], _tailBoneAxis[i]);

                //prevPoint += _tailBoneOffsets[i + 1];                            //TODO: compute offsets!
                prevPoint += rotation * _tailBoneOffsets[i + 1];                            //TODO: compute offsets!

                Debug.DrawLine(prev, prevPoint, Color.blue);
            }


            // The end of the effector
            return new PositionRotation(prevPoint, rotation);
        }


    }
}
