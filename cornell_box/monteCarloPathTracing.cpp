#include "monteCarloPathTracing.h"

double random0to1()
{
    static double invRAND_MAX = 1.0/RAND_MAX;
    return rand()*invRAND_MAX;
}


// defined by the pair of min and max 
double light_min[3] = {160, 511, 160};
double light_max[3] = {352, 511, 352};
double light_intensity_scale = 8.0; // arbitrary

double sphere_center[2][3] = { {100, 50, 100}, {350, 100, 200} };
double sphere_radius[2]    = {50, 100};



int FindNearestIntersection(double *org, double *dir, int current_object_index, double *hit_point, bool &isLight)
{
    isLight = false;

    int    nearest_object_index = -1;
    double nearest_t = 1.0e6+1;
   
    double inv_dir[3] = { 1.0/dir[0], 1.0/dir[1], 1.0/dir[2], };

    // intersection test between ray and 5 walls
    for(int i = 0; i < 5; i++){

        if(i == current_object_index) continue;

        double t;

        switch(i){
        case 0: t = -org[2]*inv_dir[2];            break;
        case 1: t = -org[0]*inv_dir[0];            break;
        case 2: t = -org[1]*inv_dir[1];            break;
        case 3: t = -(-org[0]+512.0)*-inv_dir[0];  break;
        case 4: t = -(-org[1]+512.0)*-inv_dir[1];  break;
        }


        if(t > 0.0 && t < nearest_t){
            double intersect_1, intersect_2;

            switch(i){
            case 0:
                intersect_1 = org[0]+dir[0]*t;
                intersect_2 = org[1]+dir[1]*t;
                break;
            case 1:
            case 3:
                intersect_1 = org[1]+dir[1]*t;
                intersect_2 = org[2]+dir[2]*t;
                break;
            case 2:
            case 4:
                intersect_1 = org[0]+dir[0]*t;
                intersect_2 = org[2]+dir[2]*t;
                break;
            }

            if(0.0 <= intersect_1 && intersect_1 <= 512.0 && 0.0 <= intersect_2 && intersect_2 <= 512.0){
                nearest_t = t;
                nearest_object_index = i;

                if(i == 4 && light_min[0] <= intersect_1 && intersect_1 <= light_max[0] && light_min[2] <= intersect_2 && intersect_2 <= light_max[2]){
                    isLight = true;
                }
            }
        } // if(t > nearest_t){

    } // for(int i = 0; i < 5; i++){

    // intersection test between ray and 2 spheres

    double a     = DotProduct(dir, dir);
    double inv_a = 1.0/a;
    double dot_org_dir = DotProduct(org, dir);
    double dot_org_org = DotProduct(org, org);

    for(int i = 0; i < 2; i++){
       // if(i+5 == current_object_index) continue;

        double b = 2.0*(dot_org_dir-DotProduct(dir, sphere_center[i]));
        double c = dot_org_org - 2.0*DotProduct(org, sphere_center[i]) + DotProduct(sphere_center[i], sphere_center[i]) - sphere_radius[i]*sphere_radius[i];

        double discriminant = b*b-4*a*c;

        if(discriminant >= 0.0){
            double t = (-b-sqrt(discriminant))*(0.5*inv_a);

            if(i+5 == current_object_index && fabs(t) < 1.0e-6){
                // ray passes through inside of the sphere
                return -1;
            }else if(i+5 != current_object_index){

                if(t > 0.0 && t < nearest_t){
                    nearest_t = t;
                    nearest_object_index = i+5;
                }
            }
        }
    }

    if(nearest_object_index != -1){
        for(int i = 0; i < 3; i++) hit_point[i] = org[i] + dir[i]*nearest_t;
    }

    return nearest_object_index;
}



void GetMaterialProperty(double *hit_point, int object_index, bool isLight, 
                         double *object_normal, double *object_diffuse, double *object_specular, double &object_shininess)
{
    switch(object_index){
    case 0:
        object_normal[0]  = 0.0;  object_normal[1]  = 0.0;  object_normal[2]  = 1.0; 
        
        object_diffuse[0] = object_diffuse[1] = object_diffuse[2] = 1.0;
        object_specular[0] = object_specular[1] = object_specular[2] = 0.0;
        object_shininess = 1.0;
        break;
    case 1:
        object_normal[0]  = 1.0;  object_normal[1]  = 0.0;  object_normal[2]  = 0.0;

        object_diffuse[0] = 0.3;  object_diffuse[1] = 0.8;  object_diffuse[2] = 0.3;
        object_specular[0] = object_specular[1] = object_specular[2] = 0.0;
        object_shininess = 1.0;
        break;
    case 2:
        object_normal[0]  = 0.0;  object_normal[1] = 1.0;  object_normal[2]  = 0.0; 
   
        object_diffuse[0]  = object_diffuse[1]  = object_diffuse[2] = 1.0;
        object_specular[0] = object_specular[1] = object_specular[2] = 0.0;
        object_shininess = 40.0;
        break;
    case 3:
        object_normal[0]  = -1.0;  object_normal[1]  = 0.0;  object_normal[2]  = 0.0;

        object_diffuse[0] =  0.3;  object_diffuse[1] = 0.3;  object_diffuse[2] = 0.8;
        object_specular[0] = object_specular[1] = object_specular[2] = 0.0;
        object_shininess = 1.0;
        break;
    case 4:
        object_normal[0]  = 0.0;  object_normal[1]  = -1.0;  object_normal[2]  = 0.0; 

        object_diffuse[0] = object_diffuse[1] = object_diffuse[2] = 1.0;
        object_specular[0] = object_specular[1] = object_specular[2] = 0.0;
        object_shininess = 1.0;
        break;

    case 5:
        for(int i = 0; i < 3; i++) object_normal[i] = hit_point[i]-sphere_center[0][i];
        Normalize(object_normal); 

        object_diffuse[0]  = 0.6;  object_diffuse[1]  = 0.3;  object_diffuse[2]  = 0.1;
        object_specular[0] = 0.4;  object_specular[1] = 0.3;  object_specular[2] = 0.1;
        object_shininess = 400.0;
        break;
    case 6:
        for(int i = 0; i < 3; i++) object_normal[i] = hit_point[i]-sphere_center[1][i];
        Normalize(object_normal);

        object_diffuse[0] = 0.1;  object_diffuse[1] = 0.9;  object_diffuse[2] = 0.9;
        object_specular[0] = object_specular[1] = object_specular[2] = 0.1;
        object_shininess = 1.0;
        break;
    }

    if(isLight){
        object_diffuse[0] = object_diffuse[1] = 1.0;
        object_diffuse[2] = 1.0;
    }

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

    }while( DotProduct(random_direction, normal) < 0.0); // random_direction must be directed to the direction of normal;

}



ofstream fout("temp.txt");





void AddDirectLighting(double *hit_point, double *outgoing_direction, int nearest_object_index, double *color)
{
    // shoot shadow ray (perform direct lighting)
    //double shadow_ray[3] = { light_min[0]+(light_max[0]- light_min[0])*0.5 - hit_point[0], 
    //                                                                   light_min[1] - hit_point[1], 
    //                         light_min[2]+(light_max[2]- light_min[2])*0.5 - hit_point[2] };
    double shadow_ray[3] = { light_min[0]+(light_max[0]- light_min[0])*random0to1() - hit_point[0], 
                                                                       light_min[1] - hit_point[1], 
                             light_min[2]+(light_max[2]- light_min[2])*random0to1() - hit_point[2] };
 
    double hit_light_position[3];

    Normalize(shadow_ray);

    double object_normal[3], object_diffuse[3], object_specular[3], object_shininess;

    GetMaterialProperty(hit_point, nearest_object_index, false, object_normal, object_diffuse, object_specular, object_shininess);

    int  nearest_object_index_shadow_ray; 
    bool isLight = false;



    if( (nearest_object_index_shadow_ray = FindNearestIntersection(hit_point, shadow_ray, nearest_object_index, hit_light_position, isLight)) != -1 ){
        if(nearest_object_index_shadow_ray == 4 && isLight){

            // shadow ray hits the light
            double light_normal[3], light_emission[3];

            double dummy, dummy_array[3];
            GetMaterialProperty(hit_light_position, nearest_object_index_shadow_ray, isLight, light_normal, light_emission, dummy_array, dummy);

            double r[3];
            for(int i = 0; i < 3; i++) r[i] = hit_light_position[i] - hit_point[i];

            double reflection_vector[3];

            ComputeReflectionVector(object_normal, outgoing_direction, reflection_vector);

            double diffuse_weight  = InvPI;
            double specular_weight = (object_shininess+1)*(0.5*InvPI)*pow( Max(DotProduct(shadow_ray, reflection_vector), 0.0), (int)object_shininess);

            static double light_area = (light_max[0]-light_min[0])*(light_max[2]-light_min[2]);

            double weight = light_intensity_scale * light_area * DotProduct(shadow_ray, object_normal) * -DotProduct(shadow_ray, light_normal) / DotProduct(r, r);

            for(int i = 0; i < 3; i++) color[i] = light_emission[i]*weight * (object_diffuse[i]*diffuse_weight + object_specular[i]*specular_weight); 
        }

    }

 
}




void TracePathWithImportanceSampling(double *eye, double *ray, double *color, bool isDirectLighting, double weight_of_path, int depth)
{
    double probability_to_kill = 0.0;
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
        static double probability_for_diffuse[7];
        static double inv_probability_for_diffuse[7], inv_probability_for_specular[7]; 

        static bool is_probability_diffuse_computed[7] = { false, false, false, false, false, false, false, };
        
        if(is_probability_diffuse_computed[nearest_object_index] == false){
           double sum_diffuse  =  object_diffuse[0] +  object_diffuse[1] +  object_diffuse[2];
           double sum_specular = object_specular[0] + object_specular[1] + object_specular[2];

           probability_for_diffuse[nearest_object_index] = sum_diffuse / (sum_diffuse+sum_specular);

           inv_probability_for_diffuse[nearest_object_index]  = (sum_diffuse+sum_specular) / sum_diffuse;
           if(sum_specular > 1.0e-6) inv_probability_for_specular[nearest_object_index] = (sum_diffuse+sum_specular) / sum_specular;
           else                      inv_probability_for_specular[nearest_object_index] = 0.0;

           is_probability_diffuse_computed[nearest_object_index] = true;
        }


        if(nearest_object_index == 4 && isLight){

            // light

            if( (isDirectLighting == true && depth != 1) || (isDirectLighting == false && random0to1() < 0.9) ){ // emit
                double nonBiasFactor;

                if(!isDirectLighting) nonBiasFactor = 1.11111111; // 1/0.9 = 1.11111111
                else                  nonBiasFactor = 1.0; 

                for(int i = 0; i < 3; i++) color[i] = inv_probability_to_kill * nonBiasFactor * light_intensity_scale * object_diffuse[i];

                return;

            }else{

                double incoming_direction[3], incoming_color[3];

                if( random0to1() <= probability_for_diffuse[nearest_object_index] ){
                    // evaluate diffuse

                    // importance sampling
                    GetDirectionUsingCosWeightedLambertian(hit_point, object_normal, incoming_direction);

                    double nonBiasFactor;

                    if(!isDirectLighting) nonBiasFactor = 10.0;
                    else                  nonBiasFactor = 1.0;

                    double weight = inv_probability_to_kill * inv_probability_for_diffuse[nearest_object_index] * nonBiasFactor;

                    TracePathWithImportanceSampling(hit_point, incoming_direction, incoming_color, isDirectLighting, weight_of_path*weight, depth+1);

                    for(int i = 0; i < 3; i++) color[i] = weight * incoming_color[i] * object_diffuse[i];

                }else{
                    // evaluate specular
                    GetDirectionUsingPhongBRDF(hit_point, object_normal, ray, object_shininess, incoming_direction);

                    double nonBiasFactor;

                    if(!isDirectLighting) nonBiasFactor = 10.0;
                    else                  nonBiasFactor = 1.0;

                    double weight = inv_probability_to_kill * inv_probability_for_specular[nearest_object_index] * nonBiasFactor * DotProduct(incoming_direction, object_normal);

                    TracePathWithImportanceSampling(hit_point, incoming_direction, incoming_color, isDirectLighting, weight_of_path*weight, depth+1);

                    for(int i = 0; i < 3; i++) color[i] = weight * incoming_color[i] * object_specular[i];
                }

                return;
            }

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


            if(isDirectLighting && depth == 0){
                double direct_color[3] = {0, 0, 0};
                AddDirectLighting(hit_point, ray, nearest_object_index, direct_color);

                for(int i = 0; i < 3; i++) color[i] += inv_probability_to_kill * direct_color[i];
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
    double inv_probability_to_kill = 1.0;

    if(weight_of_path < 0.01 || depth > MAX_DEPTH){ // if weight_of_path < 0.01, we stop recursion with probability of 50%
        probability_to_kill     = 0.5;
        inv_probability_to_kill = 1.0 / (1.0-probability_to_kill);

        if(true || random0to1() < probability_to_kill){
            color[0] = color[1] = color[2] = 0.0;
            return;
        }
    }

    int    nearest_object_index;
    double hit_point[3], object_normal[3], object_diffuse[3], object_specular[3], object_shininess;
    bool   isLight = false;
 

    if( (nearest_object_index = FindNearestIntersection(eye, ray, -1, hit_point, isLight)) != -1 ){

        GetMaterialProperty(hit_point, nearest_object_index, isLight, object_normal, object_diffuse, object_specular, object_shininess);

        if( nearest_object_index == 4 && isLight){

            // light

            if( (isDirectLighting == true && depth != 1) || (isDirectLighting == false && random0to1() < 0.9) ){ // emit
                double nonBiasFactor;

                if(!isDirectLighting) nonBiasFactor = 1.11111111; // 1/0.9 = 1.11111111
                else                  nonBiasFactor = 1.0; 


                for(int i = 0; i < 3; i++) color[i] = inv_probability_to_kill * nonBiasFactor * light_intensity_scale * object_diffuse[i];
                return;

            }else{

                double incoming_direction[3], incoming_color[3];

                // uniform sampling
                GetRandomDirection(object_normal, incoming_direction);

                double reflection_vector[3];

                ComputeReflectionVector(object_normal, ray, reflection_vector);

                double nonBiasFactor;
                
                if(!isDirectLighting) nonBiasFactor = 10.0;
                else                  nonBiasFactor = 1.0;  

                double weight          = inv_probability_to_kill * TwoPI*nonBiasFactor * DotProduct(incoming_direction, object_normal);
                double diffuse_weight  = InvPI;
                double specular_weight = (object_shininess+1)*(0.5*InvPI)*pow(DotProduct(incoming_direction, reflection_vector), (int)object_shininess);

                TracePathWithUniformSampling(hit_point, incoming_direction, incoming_color, isDirectLighting, weight_of_path*weight*(diffuse_weight+specular_weight), depth+1);

                for(int i = 0; i < 3; i++){
                    color[i] = weight * incoming_color[i] * (object_diffuse[i]*diffuse_weight + object_specular[i]*specular_weight);
                }

                return;
            }

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

            TracePathWithUniformSampling(hit_point, incoming_direction, incoming_color, isDirectLighting, weight_of_path*weight*(diffuse_weight+specular_weight), depth+1);

            for(int i = 0; i < 3; i++){
                // (object_diffuse[i]*diffuse_weight + object_specular[i]*specular_weight) <- This is BRDF
                color[i] = weight * incoming_color[i] * (object_diffuse[i]*diffuse_weight + object_specular[i]*specular_weight);
            }

            if(isDirectLighting && depth == 0){
                double direct_color[3] = {0, 0, 0};
                AddDirectLighting(hit_point, ray, nearest_object_index, direct_color);

                for(int i = 0; i < 3; i++) color[i] += inv_probability_to_kill * direct_color[i];
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

    for (int i = 0; i < window_height; i++){
        for (int j = 0; j < window_width; j++) {

            double ray[3] = { deltaW*(j+random0to1())-eye[0], deltaH*(i+random0to1())-eye[1], 512.0-eye[2] };
            double color[3] = { 0.0, 0.0, 0.0, };
            
            Normalize(ray);

            //TracePathWithUniformSampling(eye, ray, color, true, 1.0, 0);
            TracePathWithImportanceSampling(eye, ray, color, true, 1.0, 0);

            image[(i*window_width+j)*3]    += (GLfloat)color[0];
            image[(i*window_width+j)*3 +1] += (GLfloat)color[1];
            image[(i*window_width+j)*3 +2] += (GLfloat)color[2];
        }
    }

}

