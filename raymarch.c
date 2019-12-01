/*
 * A simple CPU raymarcher that writes the result to a BMP file
 *
 * TODOs 
 * Top TODO: make everything less hard-coded, add some kind
 *           of object data structure, allow more than one colour
 *     TODO: networked parallelisation (for clusters)
 *
 *      ...
 *
 * Bottom TODO: Global Illumination / Photon Mapping
 * Sub-Bottom TODO: Render video
 */

#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <math.h>
#include <pthread.h>

#define BITS_PER_PIXEL 24

#define UNIT_X ((vec3_f){1, 0, 0})
#define UNIT_Y ((vec3_f){0, 1, 0})
#define UNIT_Z ((vec3_f){0, 0, 1})

typedef
struct
{
    uint16_t header;
    uint16_t file_size_lower; /* Split for alignment reasons */
    uint16_t file_size_higher;
    uint16_t reserved_a;
    uint16_t reserved_b;
    uint16_t pixel_offset_lower;
    uint16_t pixel_offset_higher;
}
bmp_header;

typedef
struct
{
    uint32_t header_size;
    int32_t img_width_px;
    int32_t img_height_px;
    uint16_t num_colour_planes;
    uint16_t bits_per_pixel;
    uint32_t compression_method;
    uint32_t pixel_data_size;
    int32_t horizontal_res;
    int32_t vertical_res;
    uint32_t num_palette_colours;
    uint32_t num_important_colours;
}
dib_header;

typedef
struct
{
    uint8_t red;
    uint8_t green;
    uint8_t blue;
}
pixel;

typedef
struct
{
    float x;
    float y;
    float z;
}
vec3_f;

typedef
struct
{
    pixel *px_buf;
    int px_buf_len;
    uint32_t region_x;
    uint32_t region_y;
    uint32_t region_width;
    uint32_t region_height;
    uint32_t screen_width;
    uint32_t screen_height;
    float px_unit;
}
march_region_args;

int pack_pix_row(uint8_t *row, uint32_t row_len, pixel *image_row, uint32_t image_row_len)
{
    int i, index;
    if(row_len < (((image_row_len * BITS_PER_PIXEL) + 31) / 32) * 4)
        return -1; /* row is too small to hold the pixel row */
    else
    {
        for(i = 0; i < row_len; i++)
        {
            row[i] = 0;
        }
        index = 0;
        for(i = 0; i < image_row_len; i++)
        {
            row[index] = image_row[i].blue;
            row[index+1] = image_row[i].green;
            row[index+2] = image_row[i].red;
            index += 3;
        }
        return 0;
    }
}

uint8_t sat_255(uint32_t x)
{
    if(x > 255) return 255;
    else return (uint8_t)x;
}

float magnitude(vec3_f v)
{
    return sqrtf(v.x*v.x + v.y*v.y + v.z*v.z);
}

vec3_f add(vec3_f v1, vec3_f v2)
{
    vec3_f v3;
    v3.x = v1.x + v2.x;
    v3.y = v1.y + v2.y;
    v3.z = v1.z + v2.z;
    return v3;
}

vec3_f sub(vec3_f v1, vec3_f v2)
{
    vec3_f v3;
    v3.x = v1.x - v2.x;
    v3.y = v1.y - v2.y;
    v3.z = v1.z - v2.z;
    return v3;
}

vec3_f normalise(vec3_f v)
{
    vec3_f v_norm;
    float mag = magnitude(v);
    if(mag != 0)
    {
        v_norm.x = v.x / mag;
        v_norm.y = v.y / mag;
        v_norm.z = v.z / mag;
    }
    return v_norm;
}

vec3_f mul(vec3_f v, float scalar)
{
    vec3_f v_mul;
    v_mul.x = v.x * scalar;
    v_mul.y = v.y * scalar;
    v_mul.z = v.z * scalar;
    return v_mul;
}

vec3_f rotate_x(vec3_f v, float theta)
{
    vec3_f v_rot;
    float sine = sinf(theta);
    float cosine = cosf(theta);

    v_rot.x = v.x;
    v_rot.y = cosine * v.y - sine * v.z;
    v_rot.z = sine * v.y + cosine * v.z;
    return v_rot;
}

float dot(vec3_f v1, vec3_f v2)
{
    return (v1.x * v2.x) + (v1.y * v2.y) + (v1.z * v2.z);
}

float smin(float a, float b, float k)
{
    float h = 0.5 + (0.5 * (b - a) / k);
    h = fminf(h, 1.0);
    h = fmaxf(h, 0.0);
    return ((1.0 - h)*b + h*a) - k * h * (1.0 - h);
}

float sphere(vec3_f centre, float radius, vec3_f pos)
{
    return magnitude(sub(pos, centre)) - radius;
}

float sphere_sdf(vec3_f pos)
{
    return sphere((vec3_f){0.0, 0.0, 0.0}, 40.0, pos);
}

float cube_sdf(vec3_f pos)
{
    pos = rotate_x(pos, 0.2);
    float distx = fabsf(pos.x) - 29.0;
    float disty = fabsf(pos.y) - 29.0;
    float distz = fabsf(pos.z) - 29.0;

    if(distx > 0.0 && disty > 0.0 && distz > 0.0)
        return sqrtf(distx*distx + disty*disty + distz*distz);
    else
        return fmaxf(fmaxf(distx, disty), distz);
}

float plane_sdf(vec3_f pos)
{
    return pos.y + 35.0;
}

float curveplane_sdf(vec3_f pos)
{
    pos.y += 335.0;
    pos.x = 0.0; /* To make into cylinder */
    //pos.z -= 70.0; /* put at +ve (forward) in z */
    return magnitude(pos) - 300.0;

}

float get_sdf(vec3_f pos)
{
    return smin(sphere_sdf(pos), curveplane_sdf(pos), 20.0);
/*  return sphere_sdf(pos) - (0.0005 * ((float)rand()/(float)RAND_MAX)); - with some randomness. Ideally would want fixed sdf for a given angle
 *  from (0,0) but this is tricky */
/*    return smin(cube_sdf(pos), sphere_sdf(pos), 0.7);*/
}

void print_vec3_f(vec3_f v)
{
    printf("{ %f, %f, %f } ", v.x, v.y, v.z);
}

vec3_f march_ray(vec3_f march_pos, vec3_f march_direction)
{
    vec3_f march_step;
    vec3_f surface_normal, light_pos, light_vec, reflect_vec;
    vec3_f k_amb, k_diff, diffuse, k_spec, specular, colour;

    float sdf;
    float n_spec;

    uint32_t steps;

    light_pos = (vec3_f){80.0, 80.0, -60.0};

    march_step = normalise(march_direction);
    sdf = get_sdf(march_pos);

    steps = 0;

    while(sdf > 0.001 && steps < 2000)
    {
        march_pos = add(march_pos, mul(march_step, sdf));
        steps++;
        sdf = get_sdf(march_pos);
    }

    if(sdf < 0.001)
    {
        /* Constants */
        k_spec = (vec3_f){0.4, 0.4, 0.4};
        n_spec = 5.0;
        k_diff = (vec3_f){0.15, 0.0, 0.25};
        k_amb = (vec3_f){0.05, 0.0, 0.1};

        /* Get normal to surface */
        surface_normal.x = get_sdf(add(march_pos, mul(UNIT_X, 0.01))) - get_sdf(march_pos);
        surface_normal.y = get_sdf(add(march_pos, mul(UNIT_Y, 0.01))) - get_sdf(march_pos);
        surface_normal.z = get_sdf(add(march_pos, mul(UNIT_Z, 0.01))) - get_sdf(march_pos);
        surface_normal = normalise(surface_normal);

        /* Calculate diffuse lighting */
        light_vec = normalise(sub(light_pos, march_pos));
        diffuse = mul(k_diff, fmaxf(dot(surface_normal, light_vec), 0.0));

        /* Calculate specular lighting */
        reflect_vec = normalise(sub(march_step, mul(surface_normal, 2.0*dot(march_step, surface_normal))));
        specular = mul(k_spec, powf(fmaxf(dot(reflect_vec, light_vec), 0.0), n_spec));
 
        /* Calculate final colour */
        colour = add(specular, add(diffuse, k_amb));

        /* Saturate */
        colour.x = fminf(colour.x, 1.0);
        colour.y = fminf(colour.y, 1.0);
        colour.z = fminf(colour.z, 1.0);
    }
    else
    {
        colour = (vec3_f){0.0, 0.0, 0.0};
    }

    return colour;
}

void march(pixel *px, uint32_t x, uint32_t y, uint32_t width, uint32_t height, float px_unit)
{
    vec3_f pixel_pos, camera_pos, march_start_pos, ray_direction;
    vec3_f surface_normal, light_pos, light_vec, reflect_vec;
    vec3_f k_amb, k_diff, diffuse, k_spec, specular, colour;

    float sdf;
    uint32_t steps;

    float n_spec;

    pixel_pos.x = px_unit * ((float)x - (float)width/2.0);
    pixel_pos.y = px_unit * ((float)y - (float)height/2.0);
    pixel_pos.z = -128.0;

    camera_pos.x = 0.0;
    camera_pos.y = 0.0;
    camera_pos.z = -100000.0;

    march_start_pos = pixel_pos;

    ray_direction = normalise(sub(pixel_pos, camera_pos));

    colour = march_ray(march_start_pos, ray_direction);

    px->red = (uint8_t)(255.0*colour.x);
    px->green = (uint8_t)(255.0*colour.y);
    px->blue = (uint8_t)(255.0*colour.z);

}

/* Ray march a rectangular region of pixels. Rectangular rather than sequential was chosen as it is likely to contain the same objects in a
 * scene so future bounding-box optimisations may be easier. */
/* The region is written into px_buf row-by-row from low-to-high x and low-to-high y (i.e. [(0, 0), (1, 0), (2, 0), (0, 1), (1, 1), (2, 1), ...]
 * where a point is (x, y)).
 * This function isn't designed to write directly to the image buffer, partly because the cost of copying from px_buf to the image is small
 * compared to rendering, and partly because it makes it easier to extend to a cluster/distributed system. */
int march_region(pixel *px_buf, int px_buf_len, uint32_t region_x, uint32_t region_y, uint32_t region_width, uint32_t region_height, uint32_t screen_width, uint32_t screen_height, float px_unit)
{
    int i, j;
    if(px_buf_len < region_width*region_height)
    {   /* The provided buffer is not big enough */
        return -1;
    }
    else
    {
        for(i = 0; i < region_height; i++)
        {
            for(j = 0; j < region_width; j++)
            {
                march(&px_buf[i*region_width + j], region_x + j, region_y + i, screen_width, screen_height, px_unit);
            }
        }
    }
    return 0;
}

void *thread_march_region(void *args_struct)
{
    march_region_args *args = (march_region_args *)args_struct;
    march_region(args->px_buf, args->px_buf_len, args->region_x, args->region_y, args->region_width, args->region_height, args->screen_width, args->screen_height, args->px_unit);
    return NULL;
    /*TODO err handling passthrough */
}

int merge_region(pixel *image, int image_len, uint32_t screen_width, uint32_t screen_height, pixel *px_buf, int px_buf_len, uint32_t region_x, uint32_t region_y, uint32_t region_width, uint32_t region_height)
{
    int i, j;
    if(px_buf_len < region_width*region_height || image_len < screen_width*screen_height)
    {   /* The provided buffer is not big enough */
        return -1;
    }
    else if((region_x + region_width > screen_width) || (region_y + region_height > screen_height))
    {   /* The region is out of range of the image */
        return -1;
    }
    else
    {
        for(i = 0; i < region_height; i++)
        {
            for(j = 0; j < region_width; j++)
            {
                image[(region_y + i)*screen_width + region_x + j] = px_buf[i*region_width + j];
            }
        }
    }
    return 0;
}

void create_thread_args(march_region_args *thread_args, pixel **thread_bufs, uint32_t num_threads, uint32_t screen_width, uint32_t screen_height, float px_unit)
{
    int i;

    for (i = 0; i < num_threads; i++)
    {
        thread_args[i].screen_width = screen_width;
        thread_args[i].screen_height = screen_height;
        thread_args[i].px_unit = px_unit;
    }
    /* Divide screen into regions and allocate to threads - for now do horizontal split,
        but consider changing for more square regions in future to limit objects
        in each thread's view */
    if (screen_height >= num_threads)
    {
        for (i = 0; i < num_threads; i++)
        {
            thread_args[i].region_x = 0;
            thread_args[i].region_y = screen_height * i / num_threads;
            thread_args[i].region_width = screen_width;
            if (i == num_threads - 1) /* Last thread does leftovers */
                thread_args[i].region_height = screen_height - thread_args[i].region_y;
            else
                thread_args[i].region_height = screen_height * (i+1) / num_threads - thread_args[i].region_y;

            thread_bufs[i] = (pixel *)malloc(thread_args[i].region_width * thread_args[i].region_height * sizeof(pixel));

            if (thread_bufs[i] == NULL)
            {
                /* TODO better err handling than this */
                fprintf(stderr, "malloc of thread buffer failed\n");
                return;
            }

            thread_args[i].px_buf = thread_bufs[i];
            thread_args[i].px_buf_len = thread_args[i].region_width * thread_args[i].region_height;
        }
    }
    else
    {
        fprintf(stderr, "Image is not tall enough for this simplistic region allocation algorithm\n");
    }
}

int main(void)
{
    uint32_t num_threads = 16;
    pthread_t thread_ids[num_threads];
    march_region_args thread_args[num_threads];
    pixel *thread_px_bufs[num_threads];
    int i;

    /* TODO endianness neutrality - BMP uses little endian by default */
    bmp_header hdr;
    dib_header dib_hdr;

    int32_t height = 1080;
    int32_t width = 1920;

    pixel image[height*width];
    pixel *rend_buf;

    uint32_t px_row_size = (((width * BITS_PER_PIXEL) + 31) / 32) * 4; /* Integer ceil((width * BITS_PER_PIXEL) / 32) * 4 (for number of bytes) */

    uint32_t px_data_size = px_row_size * height;

    uint8_t pixel_row[px_row_size];

    uint32_t file_size = sizeof(hdr) + sizeof(dib_hdr) + px_data_size;
    uint32_t pixel_offset = sizeof(hdr) + sizeof(dib_hdr);

    uint32_t row_no, col_no;

    printf("Rendering:\n");
    fflush(stdout);

    create_thread_args(thread_args, thread_px_bufs, num_threads, width, height, 0.1);

    for (i = 0; i < num_threads; i++)
    {
        pthread_create(&thread_ids[i], NULL, thread_march_region, (void *)&thread_args[i]);
    }

    for (i = 0; i < num_threads; i++)
    {
        /* TODO err handling (both for threads and merging) */
        pthread_join(thread_ids[i], NULL);
        merge_region(image, width*height, width, height, thread_args[i].px_buf, thread_args[i].px_buf_len,
                thread_args[i].region_x, thread_args[i].region_y, thread_args[i].region_width, thread_args[i].region_height);
        free(thread_px_bufs[i]);
    }

    FILE *img = fopen("img.bmp", "w");

    if(img == NULL)
    {
        printf("Error opening file");
        return 1;
    }
    else
    {
        hdr.header = 'B' | ('M' << 8); /* "BM" are first 2 bytes */
        hdr.file_size_lower = file_size & 0xffff;
        hdr.file_size_higher = file_size >> 16;
        hdr.reserved_a = 0;
        hdr.reserved_b = 0;
        hdr.pixel_offset_lower = pixel_offset & 0xffff;
        hdr.pixel_offset_higher = pixel_offset >> 16;

        dib_hdr.header_size = sizeof(dib_hdr);
        dib_hdr.img_width_px = width;
        dib_hdr.img_height_px = height;
        dib_hdr.num_colour_planes = 1; /* it just is */
        dib_hdr.bits_per_pixel = 24;
        dib_hdr.compression_method = 0; /* BI_RGB aka no compression */
        dib_hdr.pixel_data_size = px_data_size;
        dib_hdr.horizontal_res = 2835;
        dib_hdr.vertical_res = 2835;
        dib_hdr.num_palette_colours = 0;
        dib_hdr.num_important_colours = 0;

        fwrite((void *)&hdr, (size_t)1, sizeof(hdr), img);
        fwrite((void *)&dib_hdr, (size_t)1, sizeof(dib_hdr), img);

        for(row_no = 0; row_no < height; row_no++)
        {
            pack_pix_row(pixel_row, px_row_size, image + (width*row_no), width);
            fwrite((void *)pixel_row, (size_t)1, (size_t)px_row_size, img);
        }

        fclose(img);

        return 0;
    }
}



