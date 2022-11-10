using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using UnityEngine;


namespace OctopusController
{
    public enum TentacleMode { LEG, TAIL, TENTACLE };

    public class MyOctopusController 
    {
        
        MyTentacleController[] _tentacles =new  MyTentacleController[4];

        Transform _currentRegion;
        Transform _target;

        Transform[] _randomTargets;// = new Transform[4];


        float _twistMin, _twistMax;
        float _swingMin, _swingMax;


        float[] _theta;
        float[] _sin;
        float[] _cos;


        #region public methods
        //DO NOT CHANGE THE PUBLIC METHODS!!

        public float TwistMin { set => _twistMin = value; }
        public float TwistMax { set => _twistMax = value; }
        public float SwingMin {  set => _swingMin = value; }
        public float SwingMax { set => _swingMax = value; }
        

        public void TestLogging(string objectName)
        {

           
            Debug.Log("hello, I am initializing my Octopus Controller in object "+objectName);

            
        }

        public void Init(Transform[] tentacleRoots, Transform[] randomTargets)
        {
            _tentacles = new MyTentacleController[tentacleRoots.Length];

            // foreach (Transform t in tentacleRoots)
            for(int i = 0;  i  < tentacleRoots.Length; i++)
            {

                _tentacles[i] = new MyTentacleController();
                _tentacles[i].LoadTentacleJoints(tentacleRoots[i],TentacleMode.TENTACLE);
                //TODO: initialize any variables needed in ccd
            }

            _randomTargets = randomTargets;
            //TODO: use the regions however you need to make sure each tentacle stays in its region

            _theta = new float[_tentacles[0].Bones.Length];
            _sin = new float[_tentacles[0].Bones.Length];
            _cos = new float[_tentacles[0].Bones.Length];
        }

              
        public void NotifyTarget(Transform target, Transform region)
        {
            _currentRegion = region;
            _target = target;
        }

        public void NotifyShoot() {
            //TODO. what happens here?
            Debug.Log("Shoot");
        }


        public void UpdateTentacles()
        {
            //TODO: implement logic for the correct tentacle arm to stop the ball and implement CCD method
            update_ccd();
        }




        #endregion


        #region private and internal methods
        //todo: add here anything that you need

        void update_ccd() 
        {
            for (int tentacleI = 0; tentacleI < _tentacles.Length; ++tentacleI)
            {              

                for (int i = _tentacles[tentacleI].Bones.Length - 2; i >= 0; i--)
                {
                    // The vector from the ith joint to the end effector
                    Vector3 r1 = (_tentacles[tentacleI].Bones[_tentacles[tentacleI].Bones.Length - 1].transform.position -
                        _tentacles[tentacleI].Bones[i].transform.position).normalized;

                    // The vector from the ith joint to the target
                    Vector3 r2 = (_randomTargets[tentacleI].position - _tentacles[tentacleI].Bones[i].transform.position).normalized;

                    // to avoid dividing by tiny numbers
                    if (r1.magnitude * r2.magnitude <= 0.001f)
                    {
                        // cos ? sin? 
                        _cos[i] = 1.0f;
                        _sin[i] = 0.0f;
                    }
                    else
                    {
                        // find the components using dot and cross product
                        //TODO4
                        float dot = Vector3.Dot(r1, r2);
                        _cos[i] = dot;
                        Vector3 cross = Vector3.Cross(r1, r2);
                        _sin[i] = cross.magnitude;
                    }


                    // The axis of rotation 
                    Vector3 axis = Vector3.Cross(r1, r2).normalized;

                    // find the angle between r1 and r2 (and clamp values if needed avoid errors)
                    _theta[i] = Mathf.Acos(Mathf.Clamp(_cos[i], -1f, 1f));

                    //Optional. correct angles if needed, depending on angles invert angle if sin component is negative
                    if (_sin[i] < 0.0f)
                        _theta[i] = -_theta[i];


                    // obtain an angle value between -pi and pi, and then convert to degrees
                    _theta[i] = _theta[i] * Mathf.Rad2Deg;

                    // rotate the ith joint along the axis by theta degrees in the world space.
                    _tentacles[tentacleI].Bones[i].transform.rotation = 
                        Quaternion.AngleAxis(_theta[i], axis) * _tentacles[tentacleI].Bones[i].transform.rotation;

                }
            }
            
            

        }


        

        #endregion






    }
}
