#include <iostream>
#include "ros/ros.h"
#include <Eigen/Dense>
//The "FlaccoAvoidance" class will be a member of the CartesianPositionController class, imported using a header file.
#include <math.h>
#include <numeric> // std::inner_product
#include <vector> // for all arrays

//use variable style obstaclePoints, rather than obstacle_points
int numberOfControlPoints = 7;
ros::Rate r(10);

class FlaccoAvoidance
{
public:
    //CallbackLivePoints not needed

    void FlaccoAvoidance()
    {
        //Parameters of the flacco paper
        float maxRate{1.0};
        float alpha{6.0};
        float c{5.0};
        float rho{0.4};
        float vMax{0.21};
        vector<float> qDotMin{-2.1750, -2.1750, -2.1750, -2.1750, -2.6100, -2.6100, -2.6100};
        vector<float> dotMax{+2.1750, +2.1750, +2.1750, +2.1750, +2.6100, +2.6100, +2.6100};

        // __init__
        vector<float> controlPoints{};
        vector<float> obstaclePoints{5.0, 5.0 ,5.0};
        vector<float> vi{0, 0, 0};
        vector<float> sphere_radiuses{0.23, 0.24, 0.2, 0.237, 0.225, 0.20, 0.27, 0.3};

        int i,j;
        float r[]={};
        float dMatrix[][]={};
        float ds[]={};
        float l2;
        float d[]={};
        float unitaryVector[]={};
        float norms[]={};
        float temp;
        float tempMax;
        float tempMin;
        int index;
        float qdotmax[]={};
        float qdotmin[]={};
        float repulsiveVector[]={};
        int m;
    }

    void getControlPoints()
    {
        controlPoints[]={};
        while(ros::ok())
        {
            try
            {
                listener.lookupTransform("/world", "/end_effector" + std::to_string(i),ros::Time(0), transform_control_points[i]);
                translation_control_points[i] << transform_control_points[i].getOrigin().getX(),
                                                transform_control_points[i].getOrigin().getY(),
                                                transform_control_points[i].getOrigin().getZ();
                position[]=translation_control_points[]
                for (i = 0; i < numberOfControlPoints; i++)
                {
                    listener.lookupTransform("/world", "/control_point" + std::to_string(i),ros::Time(0), transform_control_points[i]);
                    translation_control_points[i] << transform_control_points[i].getOrigin().getX(),
                                                    transform_control_points[i].getOrigin().getY(),
                                                    transform_control_points[i].getOrigin().getZ();
                }
                break;
                
            }
            catch(tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException)
            {
                continue;
            }
        }
        return 0;
    }

    void getDistanceVectorsEndEffectors(int r[][])//ander
    {
        for (i = 0; i < sizeof(obstaclePoints)/sizeof(obstaclePoints[0]); i++)
        {
            r[i]=obstaclePoints[i]-position[i]
        }
        return 0;
    }

    void getDistanceVectorsBody(int dMatrix[][])
    {
        // int dMatrix[][]={};
        for (i = 0; i < numberOfControlPoints; i++)
        {
            for (j = 0; j < sizeof(obstaclePoints)/sizeof(obstaclePoints[0]; i++)
            {
                dMatrix[i][j]=obstaclePoints[j]-controlPoints[i]
            }   
        }
        return 0;
    }

    void getRepulsiveVector(float r[])
    {
        getDistanceVectorsEndEffectors(ds[]);
        if (ds)
        {
            d[] = searchSmallestVector(ds[][],d[]);
            l2=sqrt(std::inner_product(d, d + sizeof(d)/sizeof(d[0]), d,  init, myaccumulator, myproduct));//np.linalg.norm
            magnitude = vMax*(1/(1+exp((l2*(2/rho)-1)*alpha)));
            unitaryVector[]=d[]/l2;
            r[] = magnitude*unitaryVector[];
        }
        else
        {
            r[] = {0, 0, 0};
        }
        
    }

    void searchSmallestVector(int vectorList[][], int smallestVector[])
    {
        for (i = 0; i < sizeof(vectorList)/sizeof(vectorList[0]; i++)
        {
            l2=sqrt(std::inner_product(vectorList[i], vectorList[i] + sizeof(vectorList[i])/sizeof(vectorList[i][0]), vectorList[i],  init, myaccumulator, myproduct));//np.linalg.norm
            norms[i] = l2;
        }
        temp = norms[0];
        index;
        for(i=0; i<sizeof(norms)/sizeof(norms[0]); i++)
        {
            if(temp>norm[i])
            {
                temp=norm[i];
                index=i;
            }
        }
        smallestVector[]=vectorList[index][];
        return 0;
    }

    void selectMostRestrictive(float qDotMaxList[], float qDotMinList[], float qdotmax[], float qdotmin[])
    {
        //changes done because this function shoud return two values hence the values are stored in the parameters instead of returning.

        qdotmax[]=qDotMax[];
        qdotmin[]=qDotMin[];
        for (i = 0; i < numberOfControlPoints; i++)
        {
            for (j = 0; j < sizeof(qdotmax)/sizeof(qdotmax[0]; j++)
            {
                maxValues[j]=qDotMaxList[i][j];
            }
            for (j = 0; j < sizeof(qdotmin)/sizeof(qdotmin[0]; j++)
            {
                minValues[j]=qDotMinList[i][j];
            }
            
            tempMax = maxValues[0];
            tempMin = minValues[0];
            for(j=0; j<sizeof(maxValues)/sizeof(maxValues[0]); j++)
            {
                if(tempMax>maxValues[j])
                {
                    tempMax=maxValues[j];
                }
                if(tempMin>minValues[j])
                {
                    tempMin=minValues[j];
                }
            }
            qdotmax[i]=tempMax;
            qdotmin[i]=tempMin
        }
    }

    void applyRestrictions(float qdotmax[], float qdotmin[])
    {
        for (i = 0; i < numberOfControlPoints; i++)
        {
            if (qDot[i]>qdotmax[i])
            {
                qDot[i]=qdotmax[i];
            }
            else if (qDot[i]<qdotmin[i])
            {
                qDot[i]=qdotmin[i];
            }
        }
    }

    void endEffectorAlgorithm(float xdDot[], float r[][])
    {
        m = sizeof(getRepulsiveVector)/sizeof(getRepulsiveVector[0];
        r[6][1]={};
        for (i = 0; i < m+3; i++)
        {
            if (i < m)
            {
			repulsiveVector[i] = getRepulsiveVector[i];
            r[i][0]=xdDot[i]-repulsiveVector[i];
		    }
		    else
            {
            r[i][0]=xdDot[i];
		    }
        }
        return 0;
    }

    void bodyAlgorithm(float qdotmax[], float qdotmin[])//ander
    {
        qdotmaxlist[]={};
        qdotminlist[]={};
        for (int i = 0; i < sizeof(getDistanceVectorsBody)/sizeof(getDistanceVectorsBody[0]; i++)
        {
            if (getDistanceVectorsBody[i])
            {
                smallestDistance[]=searchSmallestVector(getDistanceVectorsBody[i]);
            }
            else
            {
                break;
            }
            l2=sqrt(std::inner_product(smallestDistance, smallestDistance + sizeof(smallestDistance)/sizeof(smallestDistance[0]), smallestDistance,  init, myaccumulator, myproduct));//np.linalg.norm
            distanceNorm=l2-sphere_radiuses[i];
            unitaryVector[]=smallestDistance[]/distanceNorm;
            f=1/(1+exp((distanceNorm*2/rho)-1)*alpha);

            jacobianMessage=getJacobian(q, 'control_point{}'.format(i));
            float j[]=jacobianMessage.j.j;
            j.reshape({Jacobian_message.J.rows, Jacobian_message.J.columns});

            s=
        }
        
    }

    void goToPoint(int positionDesired[])
    {
        for (int i = 0; i < sizeof(positionDesired)/sizeof(positionDesired[0]; i++)
        {
            error[i]=positionDesired[i]-position[i];
        }
        n=sizeof(error)/sizeof(error[0]
        while (sqrt(std::inner_product(error, error + n, error,  iniit, myaccumulator, myproduct))>errorThreshold)
        {
            getControlPoints();
            xdDot=computeCommandVelocity(positionDesired);
            xcDot=xdDot;
            qDot=computeCommandQDot(xcDot);
            bodyAlgorithm(qdotmax,qdotmin);
            applyRestrictions(qdotax,qdotmin);
            pandaController.sendVelocities(qDot);
            r.sleep();
        }
        ros::shutdown();
    }
};

int main(int argc, char const *argv[])
{
    std::cout << "Hello World" << std::endl;
    return 0;
}
