#include "Triangle.hpp"
#include "rasterizer.hpp"
#include <eigen3/Eigen/Eigen>
#include <iostream>
#include <opencv2/opencv.hpp>
#include <cmath>
// add some other header files you need
#include <vector>

constexpr double MY_PI = 3.1415926;

Eigen::Matrix4f get_view_matrix(Eigen::Vector3f eye_pos)
{
    Eigen::Matrix4f view = Eigen::Matrix4f::Identity();

    Eigen::Matrix4f translate;
    translate << 1, 0, 0, -eye_pos[0], 
                 0, 1, 0, -eye_pos[1], 
                 0, 0, 1, -eye_pos[2], 
                 0, 0, 0, 1;

    view = translate * view;
    // std::clog << "view" << std::endl << view << std::endl;  // check data

    return view;
}


Eigen::Matrix4f get_model_matrix(float rotation_angle, Eigen::Vector3f T, Eigen::Vector3f S, Eigen::Vector3f P0, Eigen::Vector3f P1)
{

    //Step 1: Build the Translation Matrix M_trans:

    //Step 2: Build the Scale Matrix S_trans:

    //Step 3: Implement Rodrigues' Rotation Formular, rotation by angle theta around axix u, then get the model matrix
	// The axis u is determined by two points, u = P1-P0: Eigen::Vector3f P0 ,Eigen::Vector3f P1  
    // Create the model matrix for rotating the triangle around a given axis. // Hint: normalize axis first

	//Step 4: Use Eigen's "AngleAxisf" to verify your Rotation
	//

    Eigen::Matrix4f M_trans;
    M_trans << 1, 0, 0, T[0], 
               0, 1, 0, T[1], 
               0, 0, 1, T[2], 
               0, 0, 0, 1;

    Eigen::Matrix4f S_trans;
    S_trans << S[0], 0, 0, 0, 
               0, S[1], 0, 0, 
               0, 0, S[2], 0, 
               0, 0, 0, 1;

    Eigen::Vector3f u = P1 - P0;
    u.normalize();


    Eigen::Matrix3f R;
    
    Eigen::Matrix3f u_matrix;

    u_matrix << 0,-u[2],u[1],
                u[2],0,-u[0],
                -u[1],u[0],0;
    
    rotation_angle = rotation_angle / 180.0 * MY_PI;

    R = cos(rotation_angle) * Eigen::Matrix3f::Identity() + (1 - cos(rotation_angle)) * u * u.transpose() + sin(rotation_angle) * u_matrix;


    Eigen::AngleAxisf rotation_vector(rotation_angle, Vector3f(u[0], u[1], u[2]));  
	Eigen::Matrix3f rotation_m;
	rotation_m = rotation_vector.toRotationMatrix();

    std::cout << "the diff of rotation matrix is: "<< std::endl;
    for(int i=0; i<3; i++){
        for(int j=0; j<3; j++){
            std::cout<< R(i*3 + j) - rotation_m(i*3 + j) << ", ";
        }
        std::cout<<std::endl;
    }

    Eigen::Matrix4f R_extend;

    //std::cout << "checkpoint1" << std::endl;

    R_extend << R(0),R(1),R(2),0,
                R(3),R(4),R(5),0,
                R(6),R(7),R(8),0,
                0,0,0,1;


    //std::cout << "checkpoint2" << std::endl;

    Eigen::Matrix4f model;

    model = S_trans * R_extend * M_trans;

    //std::cout << "checkpoint3" << std::endl;

    std::clog << "model" << std::endl << model << std::endl; //check


	return model;
}



Eigen::Matrix4f get_projection_matrix(float eye_fov, float aspect_ratio,
                                      float zNear, float zFar)
{
    // Implement this function


    // TODO: Implement this function
    // Create the projection matrix for the given parameters.
    // Then return it.

    // frustum -> cubic

    // orthographic projection

    // squash all transformations

    // std::clog << "projection" << std::endl << projection << std::endl; //check


    float fov_radian = eye_fov / 180  * MY_PI;
    
    float top = zNear * tan(fov_radian / 2);
    float bottom = - top;

    float right = top * aspect_ratio;
    float left = -right;
    
    
    Eigen::Matrix4f projection;

    projection << 2/(right - left), 0, (right + left)/(right - left),0,
                  0, 2/(top - bottom), (top + bottom)/(top - bottom),0,
                  0, 0, -(zFar + zNear)/(zFar - zNear), -2 * zFar / (zFar - zNear),
                  0, 0, -1, 0;

    std::clog << "projection" << std::endl << projection << std::endl; //check

    return projection;
}

int main(int argc, const char** argv)
{
    float angle = 0;
    bool command_line = false;
    std::string filename = "result";

    if (argc >= 3) {
        command_line = true;
        angle = std::stof(argv[2]); // -r by default
        if (argc == 4) {
            filename = std::string(argv[3]);
        }
        else
            return 0;
    }

    rst::rasterizer r(1024, 1024);
    // define your eye position "eye_pos" to a proper position

    // define a triangle named by "pos" and "ind"

    Eigen::Vector3f eye_pos = {0,0,100};

    std::vector<Eigen::Vector3f> pos {{1,0,-2},{0,2,-2},{-3,0,-4}};

    std::vector<Eigen::Vector3i> ind{{0, 1, 2}};




    auto pos_id = r.load_positions(pos);
    auto ind_id = r.load_indices(ind);

    int key = 0;
    int frame_count = 0;

    // added parameters for get_projection_matrix(float eye_fov, float aspect_ratio,float zNear, float zFar)
    
    Eigen::Vector3f T {1,1,3};
    Eigen::Vector3f S {1,1,1};
    Eigen::Vector3f P0 {0,0,0};
    Eigen::Vector3f P1 {1,1,1};

    float eye_fov = 45;
    float aspect_ratio = 1;
    float zNear = 0.1;
    float zFar = 100;



    // Eigen::Vector3f axis(0, 0, 1);
    Eigen::Vector3f axis(1, 0, 0);

/*
    if (command_line) {
        r.clear(rst::Buffers::Color | rst::Buffers::Depth);

        r.set_model(get_model_matrix(angle, T, S, P0, P1));
        r.set_view(get_view_matrix(eye_pos));
        r.set_projection(get_projection_matrix(eye_fov, aspect_ratio, zNear, zFar));

        r.draw(pos_id, ind_id, rst::Primitive::Triangle);
        cv::Mat image(1024, 1024, CV_32FC3, r.frame_buffer().data());
        image.convertTo(image, CV_8UC3, 1.0f);

        cv::imwrite(filename, image);

        return 0;
    }
*/

    while (key != 27) {
        r.clear(rst::Buffers::Color | rst::Buffers::Depth);

        r.set_model(get_model_matrix(angle, T, S, P0, P1));
        r.set_view(get_view_matrix(eye_pos));
        r.set_projection(get_projection_matrix(eye_fov, aspect_ratio, zNear, zFar));

        r.draw(pos_id, ind_id, rst::Primitive::Triangle);

        cv::Mat image(1024, 1024, CV_32FC3, r.frame_buffer().data());
        image.convertTo(image, CV_8UC3, 1.0f);
        cv::imshow("image", image);
        key = cv::waitKey(10);

        cv::imwrite("../" + filename + std::to_string(frame_count) + ".png", image);

        std::cout << "frame count: " << frame_count++ << '\n';
        std::clog << "angle: " << angle << std::endl;
    

        if (key == 'a') {
            angle += 10;
        }
        else if (key == 'd') {
            angle -= 10;
        }
    }

    return 0;
}
