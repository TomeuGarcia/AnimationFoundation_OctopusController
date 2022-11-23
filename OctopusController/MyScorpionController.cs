using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using UnityEngine;


namespace OctopusController
{
  
    public class MyScorpionController
    {
        //TAIL
        Transform tailTarget;
        Transform tailEndEffector;
        MyTentacleController _tail;
        float animationRange;

        //LEGS
        Transform[] legTargets;
        Transform[] legFutureBases;
        MyTentacleController[] _legs = new MyTentacleController[6];

        Vector3[] bonePositionCopy;
        float[][] legsDistances;
        bool doneFABRIK;

        
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

            legFutureBases = LegFutureBases;
            legTargets = LegTargets;
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

                if (i > 0)
                {
                    _tailBoneAxis[i] = _tail.Bones[i].right;
                    _tailBoneAngles[i] = _tail.Bones[i].localEulerAngles.x;
                    _tailBoneOffsets[i] = Quaternion.Inverse(_tail.Bones[i-1].rotation) * (_tail.Bones[i].position - _tail.Bones[i - 1].position);
                }
                else
                {
                    _tailBoneAxis[i] = _tail.Bones[i].forward; // Allows tail to rotate sideways
                    _tailBoneAngles[i] = _tail.Bones[i].localEulerAngles.z;
                    _tailBoneOffsets[i] = _tail.Bones[i].position;
                }

            }            


            _errorFunction = DistanceFromTarget;

            tailEndEffector = _tail.Bones[_tail.Bones.Length - 1];
        }

        //TODO: Check when to start the animation towards target and implement Gradient Descent method to move the joints.
        public void NotifyTailTarget(Transform target)
        {

        }

        //TODO: Notifies the start of the walking animation
        public void NotifyStartWalk()
        {

        }

        //TODO: create the apropiate animations and update the IK from the legs and tail

        public void UpdateIK()
        {
 
        }
        #endregion


        #region private
        //TODO: Implement the leg base animations and logic
        private void updateLegPos()
        {
            //check for the distance to the futureBase, then if it's too far away start moving the leg towards the future base position
            //
        }
        //TODO: implement Gradient Descent method to move tail if necessary
        private void updateTail()
        {

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

                Vector3 loclaEulerAngles = _tail.Bones[i].localEulerAngles;
                if (i == 0)
                {
                    _tail.Bones[i].localEulerAngles =
                        new Vector3(loclaEulerAngles.x, loclaEulerAngles.y, 0) + new Vector3(0, 0, _tailBoneAngles[i]);
                }
                else
                {
                    _tail.Bones[i].localEulerAngles =
                        new Vector3(0, loclaEulerAngles.y, loclaEulerAngles.z) + new Vector3(_tailBoneAngles[i], 0, 0);
                }

            }
        }
        //TODO: implement fabrik method to move legs 
        private void updateLegs()
        {

        }
        #endregion
    }
}
