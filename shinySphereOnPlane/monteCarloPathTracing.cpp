#include "monteCarloPathTracing.h"


double random0to1()
{
    static double invRAND_MAX = 1.0/RAND_MAX;
    return rand()*invRAND_MAX;
}


double sphere_center[3][3] = { {100, 50, 300}, {380, 100, 250}, {230, 40, 450} };
double sphere_radius[3]    = {50, 100, 40};

double cylinder_center[3] = { 256, 0, 256 };
double cylinder_radius = 600;


int FindNearestIntersection(double *org, double *dir, int current_object_index, double *hit_point, bool &isLight)
{
    isLight = false;

    int    nearest_object_index = -1;
    double nearest_t = 1.0e6+1;
   
    double inv_dir[3] = { 1.0/dir[0], 1.0/dir[1], 1.0/dir[2], };

    // intersection test between ray and table
    if(current_object_index != 0){

        double t = -org[1]*inv_dir[1];
        double intersect_1, intersect_2;

        intersect_1 = org[0]+dir[0]*t;
        intersect_2 = org[2]+dir[2]*t;

        if(t > 0.0 && t < nearest_t){
            if(0.0 <= intersect_1 && intersect_1 <= 512.0 && 0.0 <= intersect_2 && intersect_2 <= 512.0){
                nearest_t = t;
                nearest_object_index = 0;
            }
        }
    } // if(current_object_index != 0){


    // intersection test between ray and 2 spheres

    double a     = DotProduct(dir, dir);
    double inv_a = 1.0/a;
    double dot_org_dir = DotProduct(org, dir);
    double dot_org_org = DotProduct(org, org);

    for(int i = 0; i < 3; i++){
       // if(i+1 == current_object_index) continue;


        double b = 2.0*(dot_org_dir-DotProduct(dir, sphere_center[i]));
        double c = dot_org_org - 2.0*DotProduct(org, sphere_center[i]) + DotProduct(sphere_center[i], sphere_center[i]) - sphere_radius[i]*sphere_radius[i];

        double discriminant = b*b-4*a*c;

        if(discriminant >= 0.0){
            double t = (-b-sqrt(discriminant))*(0.5*inv_a);

            if(i+1 == current_object_index && fabs(t) < 1.0e-6){
                // ray passes through inside of the sphere
                return -1;
            }else if(i+1 != current_object_index){

                if(t > 0.0 && t < nearest_t){
                    nearest_t = t;
                    nearest_object_index = i+1;
                }
            }
        }

    }


    if(nearest_object_index == -1){
        // ray does not hit any objects
        // ray hits with cylinder describing environment
        
        // perform intersecton between ray and circle that is the slice of the cylinder
        double a = dir[0]*dir[0] + dir[2]*dir[2];
        double b = 2.0 * (dir[0]*(org[0]-cylinder_center[0]) + dir[2]*(org[2]-cylinder_center[2]));
        double c = (org[0]-cylinder_center[0])*(org[0]-cylinder_center[0]) + (org[2]-cylinder_center[2])*(org[2]-cylinder_center[2]) - cylinder_radius*cylinder_radius;

        double discriminant = b*b-4*a*c;

        // discriminant should be always positive in this case.
        nearest_t = (-b+sqrt(discriminant)) / (2.0*a);

        nearest_object_index = 100;
    }

    for(int i = 0; i < 3; i++) hit_point[i] = org[i] + dir[i]*nearest_t;


    return nearest_object_index;
}



void GetMaterialProperty(double *hit_point, int object_index, bool isLight, 
                         double *object_normal, double *object_diffuse, double *object_specular, double &object_shininess)
{
    int pixel_index, pixel_coord_x, pixel_coord_y;
    


    switch(object_index){
    case 0:
        object_normal[0]  = 0.0;  object_normal[1] = 1.0;  object_normal[2]  = 0.0; 

        pixel_coord_x = hit_point[0]*0.5;
        pixel_coord_y = hit_point[2]*0.5;

        if(pixel_coord_x < 0)                          pixel_coord_x = 0;
        else if(pixel_coord_x >= table_texture_width)  pixel_coord_x = table_texture_width-1;
        if(pixel_coord_y < 0)                          pixel_coord_y = 0;
        else if(pixel_coord_y >= table_texture_height) pixel_coord_y = table_texture_height-1;

        pixel_index = (pixel_coord_y*table_texture_width + pixel_coord_x) * 3;

        object_diffuse[0] = table_texture[pixel_index+0]*0.6;
        object_diffuse[1] = table_texture[pixel_index+1]*0.6;
        object_diffuse[2] = table_texture[pixel_index+2]*0.6;

        object_specular[0] = object_specular[1] = object_specular[2] = 0.4;
        object_shininess = 500.0;
        break;
    case 1:
        for(int i = 0; i < 3; i++) object_normal[i] = hit_point[i]-sphere_center[0][i];
        Normalize(object_normal);

        object_diffuse[0] = 0.1;  object_diffuse[1] = 0.1;  object_diffuse[2] = 0.3;
        object_specular[0] = 0.1; object_specular[1] = 0.7; object_specular[2] = 0.7;
        object_shininess = 20000.0;
        break;
    case 2:
        for(int i = 0; i < 3; i++) object_normal[i] = hit_point[i]-sphere_center[1][i];
        Normalize(object_normal);

        object_diffuse[0]  = 0.1;  object_diffuse[1]  = 0.1;  object_diffuse[2]  = 0.1;
        object_specular[0] = 0.9;  object_specular[1] = 0.9;  object_specular[2] = 0.9;
        object_shininess = 500.0;
        break;
    case 3:
        for(int i = 0; i < 3; i++) object_normal[i] = hit_point[i]-sphere_center[2][i];
        Normalize(object_normal);

        object_diffuse[0]  = 0.2;  object_diffuse[1]  = 0.2;  object_diffuse[2]  = 0.2;
        object_specular[0] = 0.8;  object_specular[1] = 0.2;  object_specular[2] = 0.8;
        object_shininess = 20000.0;
        break;
    default:
        // hit cylinder. Retrieve environment information
        double theta = -atan2(-(hit_point[2]-cylinder_center[2]), (hit_point[0]-cylinder_center[0])) + PI;

        int pixel_coord_x = (theta*(0.5*InvPI)) * png_width;
        int pixel_coord_y = (hit_point[1]+300)*0.5;


        if(pixel_coord_x < 0)                pixel_coord_x = 0;
        else if(pixel_coord_x >= png_width)  pixel_coord_x = png_width-1;
        if(pixel_coord_y < 0)                pixel_coord_y = 0;
        else if(pixel_coord_y >= png_height-50){
            // pick up random one to eliminate artifacts due to empty region of top of cylinder
            pixel_coord_y = png_height-1-100*random0to1();
        }

        int pixel_index = (pixel_coord_y*png_width + pixel_coord_x) * 3;

        object_diffuse[0] = png_data[pixel_index+0];
        object_diffuse[1] = png_data[pixel_index+1];
        object_diffuse[2] = png_data[pixel_index+2];

        object_specular[0] = object_specular[1] = object_specular[2] = 0.0;
        object_shininess = 1.0;
    }

    //if(isLight){
    //    object_diffuse[0] = object_diffuse[1] = 1.0;
    //    object_diffuse[2] = 1.0;
    //}
        
}




void GetRandomDirection(double *normal, double *random_direction)
{
    do{
        for(int i = 0; i < 3; i++) random_direction[i] = (random0to1()-0.5)*2.0;
    }while( DotProduct(random_direction, random_direction) > 1.0);

    Normalize(random_direction);

    if( DotProduct(random_direction, normal) < 0.0){ 
        // flip direction
        for(int i = 0; i < 3; i++) random_direction[i] = -random_direction[i];
    }

}

void GetDirectionUsingCosWeightedLambertian(double *point, double *normal, double *random_direction)
{
    // "outgoing" in a physical sense
    // "outgoing_direction" is actually an incoming direction in path tracing

    double theta = acos( sqrt( random0to1() ) );
    double phi   = random0to1()*TwoPI;

    // temporary created rondom direction
    double temp[3] = { sin(theta)*cos(phi), sin(theta)*sin(phi), cos(theta) };

    // rotate "temp" such that z-axis used for sampling is aligned with "normal"
    double angle_between = acos(normal[2]);
    double rotation_axis[3] = { -normal[1], normal[0], 0.0 };

    Normalize(rotation_axis);
    
    RotateAroundAxis(rotation_axis, angle_between, temp, random_direction);
}

void GetDirectionUsingPhongBRDF(double *point, double *normal, double *outgoing_direction, double shininess, double *random_direction)
{
    // "outgoing" in a physical sense
    // "outgoing_direction" is actually an incoming direction in path tracing

    double reflection_vector[3];

    ComputeReflectionVector(normal, outgoing_direction, reflection_vector);

    do{
        double theta = acos( pow( random0to1(), 1.0/(shininess+1.0) ) );
        double phi   = random0to1()*TwoPI;

        // temporary created rondom direction
        double temp[3] = { sin(theta)*cos(phi), sin(theta)*sin(phi), cos(theta) };

        // rotate "temp" such that z-axis used for sampling is aligned with "reflection_vector"
        double angle_between = acos(reflection_vector[2]);
        double rotation_axis[3] = { -reflection_vector[1], reflection_vector[0], 0.0 };

        Normalize(rotation_axis);

        RotateAroundAxis(rotation_axis, angle_between, temp, random_direction);

    }while( DotProduct(random_direction, normal) < 0.0); // random_direction must be directed to the direction of normal

}



void TracePathWithImportanceSampling(double *eye, double *ray, double *color, bool isDirectLighting, double weight_of_path, int depth)
{
    double probability_to_kill     = 0.0;
    double inv_probability_to_kill = 1.0;

    if(weight_of_path < 0.01 || depth > MAX_DEPTH){ // if weight_of_path < 0.01, we stop recursion with probability of 50%
        probability_to_kill     = 0.5;
        inv_probability_to_kill = 1.0 / (1.0-probability_to_kill);

        if(random0to1() < probability_to_kill){
            color[0] = color[1] = color[2] = 0.0;
            return;
        }
    }


    int    nearest_object_index;
    double hit_point[3], object_normal[3], object_diffuse[3], object_specular[3], object_shininess;
    bool   isLight = false;
 

    if( (nearest_object_index = FindNearestIntersection(eye, ray, -1, hit_point, isLight)) != -1 ){

        GetMaterialProperty(hit_point, nearest_object_index, isLight, object_normal, object_diffuse, object_specular, object_shininess);

        // precompute probability to shoot diffuse ray and specular ray for each material 
        static double probability_for_diffuse[5];
        static double inv_probability_for_diffuse[5], inv_probability_for_specular[5]; 

        static bool is_probability_diffuse_computed[5] = { false, false, false, false, false,  };

        if(is_probability_diffuse_computed[nearest_object_index] == false){
            double sum_diffuse  =  object_diffuse[0] +  object_diffuse[1] +  object_diffuse[2];
            double sum_specular = object_specular[0] + object_specular[1] + object_specular[2];

            probability_for_diffuse[nearest_object_index] = sum_diffuse / (sum_diffuse+sum_specular);

            inv_probability_for_diffuse[nearest_object_index]  = (sum_diffuse+sum_specular) / sum_diffuse;
            if(sum_specular > 1.0e-6) inv_probability_for_specular[nearest_object_index] = (sum_diffuse+sum_specular) / sum_specular;
            else                      inv_probability_for_specular[nearest_object_index] = 0.0;

            is_probability_diffuse_computed[nearest_object_index] = true;
        }

        if( nearest_object_index == 100){

            // light
            double light_intensity_scale;

            if(depth == 0) light_intensity_scale = 1.0;
            else           light_intensity_scale = 5.5;

            for(int i = 0; i < 3; i++) color[i] = inv_probability_to_kill * light_intensity_scale * object_diffuse[i];

            return;

        }else{
            // not light

            double incoming_direction[3], incoming_color[3];

            if( random0to1() <= probability_for_diffuse[nearest_object_index] ){
                // evaluate diffuse

                GetDirectionUsingCosWeightedLambertian(hit_point, object_normal, incoming_direction);

                double weight = inv_probability_to_kill * inv_probability_for_diffuse[nearest_object_index];

                TracePathWithImportanceSampling(hit_point, incoming_direction, incoming_color, isDirectLighting, weight_of_path*weight, depth+1);

                for(int i = 0; i < 3; i++) color[i] = weight * incoming_color[i]*object_diffuse[i];

            }else{
                // evaluate specular

                GetDirectionUsingPhongBRDF(hit_point, object_normal, ray, object_shininess, incoming_direction);

                double weight = inv_probability_to_kill * inv_probability_for_specular[nearest_object_index] * DotProduct(incoming_direction, object_normal);

                TracePathWithImportanceSampling(hit_point, incoming_direction, incoming_color, isDirectLighting, weight_of_path*weight, depth+1);

                for(int i = 0; i < 3; i++) color[i] = weight * incoming_color[i]*object_specular[i];
            }


            return;
        }
    
    }else{ // ray does not hit any objects
        color[0] = color[1] = color[2] = 0.0;
        return;
    }

}


void TracePathWithUniformSampling(double *eye, double *ray, double *color, bool isDirectLighting, double weight_of_path, int depth)
{

    double probability_to_kill = 0.0;

    if(weight_of_path < 0.01 || depth > MAX_DEPTH){ // if weight_of_path < 0.01, we stop recursion with probability of 50%
        probability_to_kill = 0.5;

        if(random0to1() < probability_to_kill){
            color[0] = color[1] = color[2] = 0.0;
            return;
        }
    }

    double inv_probability_to_kill = 1.0 / (1.0-probability_to_kill);


    int    nearest_object_index;
    double hit_point[3], object_normal[3], object_diffuse[3], object_specular[3], object_shininess;
    bool   isLight = false;
 

    if( (nearest_object_index = FindNearestIntersection(eye, ray, -1, hit_point, isLight)) != -1 ){

        GetMaterialProperty(hit_point, nearest_object_index, isLight, object_normal, object_diffuse, object_specular, object_shininess);

        if( nearest_object_index == 100){ 

            // light
            double light_intensity_scale;

            if(depth == 0) light_intensity_scale = 1.0;
            else           light_intensity_scale = 5.5;

            for(int i = 0; i < 3; i++) color[i] = inv_probability_to_kill * light_intensity_scale * object_diffuse[i];

            return;

        }else{
            // not light

            double incoming_direction[3], incoming_color[3];

            // uniform sampling
            GetRandomDirection(object_normal, incoming_direction);

            double reflection_vector[3];

            ComputeReflectionVector(object_normal, ray, reflection_vector);

            double weight          = inv_probability_to_kill * TwoPI * DotProduct(incoming_direction, object_normal);
            double diffuse_weight  = InvPI;
            double specular_weight = (object_shininess+1)*(0.5*InvPI)*pow(DotProduct(incoming_direction, reflection_vector), (int)object_shininess);

            TracePathWithUniformSampling(hit_point, incoming_direction, incoming_color, isDirectLighting, 
                                         weight_of_path*weight*(diffuse_weight+specular_weight), depth+1);

            for(int i = 0; i < 3; i++){
                // (object_diffuse[i]*diffuse_weight + object_specular[i]*specular_weight) <- This is BRDF
                color[i] = weight * incoming_color[i] * (object_diffuse[i]*diffuse_weight + object_specular[i]*specular_weight);
            }

            return;
        }
    
    }else{ // ray does not hit any objects
        color[0] = color[1] = color[2] = 0.0;
        return;
    }

}

void PathTracing(double *eye, GLfloat *image)
{
    static double deltaW = 512.0/window_width;
    static double deltaH = 512.0/window_height;

    double original_eye[3];
    double original_center2[3];

    for(int k = 0; k < 3; k++) original_eye[k]     = eye[k];
    for(int k = 0; k < 3; k++) original_center2[k] = sphere_center[2][k];

    
    for (int i = 0; i < window_height; i++){
        for (int j = 0; j < window_width; j++) {

            double ray[3] = { deltaW*(j+random0to1())-eye[0], deltaH*(i+random0to1())-eye[1], 512.0-eye[2] };
            double color[3] = { 0.0, 0.0, 0.0, };

            Normalize(ray);
            
            // apply depth of field
            eye[2]              -= random0to1()*10.0;  

            // apply motion blur
            double rand = random0to1();
            sphere_center[2][1] -= rand*rand*20.0;
            sphere_center[2][0] += rand*10.0;
            sphere_center[2][2] -= rand*10.0;

            //TracePathWithUniformSampling(eye, ray, color, false, 1.0, 0);
            TracePathWithImportanceSampling(eye, ray, color, false, 1.0, 0);


            for(int k = 0; k < 3; k++) eye[k]              = original_eye[k];
            for(int k = 0; k < 3; k++) sphere_center[2][k] = original_center2[k];

            image[(i*window_width+j)*3]    += (GLfloat)color[0];
            image[(i*window_width+j)*3 +1] += (GLfloat)color[1];
            image[(i*window_width+j)*3 +2] += (GLfloat)color[2];
        }
    }

}

