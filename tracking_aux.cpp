#include "tracking_aux.h"


// Gets video
void	OnMouse(int event,
				int x,
				int y,
				int,
				void* test)
{
	int *mouse_coords = (int*) test;

	if(event == CV_EVENT_LBUTTONDOWN)
	{
		mouse_coords[0] = x;
		mouse_coords[1] = y;

		printf("Coords selected!\n");
	}    
}

void	LoadParameters(std::string path,
                       int *size_template_x,
					   int *size_template_y,
					   int *n_ctrl_pts_x,
					   int *n_ctrl_pts_y,
					   int *n_ctrl_pts_xi,
					   int *n_ctrl_pts_yi,
					   float *lambda,
					   int *size_bins,
					   float *epsilon,
					   int *n_bins,
					   int *n_max_iters,
					   float *confidence_thres,
					   int *percentage_active,
					   std::string *filename,
					   int *isgrayscale,
					   int *interp)
{
    cv::FileStorage fs(path, cv::FileStorage::READ);

	*size_template_x = fs["size_template_x"];
	*size_template_y = fs["size_template_y"];
	*size_bins = fs["size_bins"];
	*epsilon = fs["epsilon"];
	*n_bins = fs["n_bins"];
	*n_max_iters = fs["n_max_iters"];
	*confidence_thres = fs["confidence_thres"];
	*percentage_active = fs["percentage_active"];
    *filename = (std::string) fs["filename"];
	*isgrayscale = fs["isgrayscale"];
	*interp = fs["interp"];
	*n_ctrl_pts_x = fs["n_ctrl_pts_x"];
	*n_ctrl_pts_y = fs["n_ctrl_pts_y"];
	*n_ctrl_pts_xi = fs["n_ctrl_pts_x_illum"];
	*n_ctrl_pts_yi = fs["n_ctrl_pts_y_illum"];
	*lambda = fs["lambda_tps"];
}

void	DefineTemplate(cv::Mat *Template,
	                   cv::Mat *ICur,
					   int *coords,
					   int isgrayscale)
{	
	/*coords[0] = ICur->cols/2;
	coords[1] = ICur->rows/2;*/

	printf("> Coordinates: %d %d \n", coords[0], coords[1]);

	if(isgrayscale)
	{
		for(int i=0;i<Template->rows;i++)
			for(int j=0;j<Template->cols;j++)
			{
				Template->at<uchar>(i, j) = ICur->at<uchar>(coords[1]-Template->rows/2+i, coords[0]-Template->cols/2+j);
			}
	}
	else
	{
		for(int i=0;i<Template->rows;i++)
			for(int j=0;j<Template->cols;j++)
			{
				Template->ptr<uchar>(i)[3*j] =   ICur->ptr<uchar>(coords[1]-Template->rows/2+i)[ (coords[0]-Template->cols/2+j)*3 ];
				Template->ptr<uchar>(i)[3*j+1] = ICur->ptr<uchar>(coords[1]-Template->rows/2+i)[ (coords[0]-Template->cols/2+j)*3 +1];
				Template->ptr<uchar>(i)[3*j+2] = ICur->ptr<uchar>(coords[1]-Template->rows/2+i)[ (coords[0]-Template->cols/2+j)*3 +2];
			}
	}
}
