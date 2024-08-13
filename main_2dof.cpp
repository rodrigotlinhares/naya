#define	 USE_VIDEO

#include <stdio.h>
#include "naya.h"
#include "tracking_aux.h"

void	GetFrame(cv::VideoCapture *capture, cv::Mat *ICur, cv::Mat *ICur_raw, int isgrayscale)
{
	*capture >> *ICur_raw;
	
	if(ICur_raw->channels() > 1 && isgrayscale == 1)
	{
		cv::cvtColor(*ICur_raw, *ICur, CV_BGR2GRAY);
	}
	else
		*ICur = *ICur_raw;
}

int main(void)
{
	// Tracking parameters
	std::string	filename;

	int		isgrayscale,	
			size_template_x, 
			size_template_y,
			size_bins,
			percentage_active,
			n_bins,
			n_max_iters,
			interp,
			coords[2] = {0},
            n_ctrl_pts_x,
            n_ctrl_pts_y,
            n_ctrl_pts_xi,
            n_ctrl_pts_yi;

	float	lambda,
			epsilon,
			confidence_thres,
		    parameters[9];

	double	clock_start,
			clock_stop,
			tick = cvGetTickFrequency();
			
	cv::Mat ICur,
		    ICur_raw,
		    Template;
	
	naya tracker; // main Naya class

	// Loading Naya parameters from file
    LoadParameters("../parameters.yml",
                   &size_template_x,
				   &size_template_y,
				   &n_ctrl_pts_x,
                   &n_ctrl_pts_y,
                   &n_ctrl_pts_xi,
                   &n_ctrl_pts_yi,
                   &lambda,
				   &size_bins,
				   &epsilon,
				   &n_bins,
				   &n_max_iters,
				   &confidence_thres,
				   &percentage_active,
				   &filename,
				   &isgrayscale,
				   &interp);

	// Are we using active pixels?
	int	n_active_pixels = cvRound((float)(size_template_x*size_template_y)*(float)(percentage_active)/100);

    // Temporary
    // So I don't really think it's worth implementing active pixel selection for the 2DOF model!
    n_active_pixels = size_template_x*size_template_y;
	
	// Some allocations
	if(isgrayscale)
		Template.create(size_template_y, size_template_x, CV_8UC1);
	else
		Template.create(size_template_y, size_template_x, CV_8UC3);

#ifdef USE_VIDEO
	
	// Loading video
	printf("> Loading video ...\n");
    cv::VideoCapture capture(0);
	if(!capture.isOpened())
	{
        printf("* Ops! what happened to the video?");
		exit(0);
	}
	printf("> Video loaded...\n");
	
	// Selecting template
	while(coords[0] == 0)
	{
		// Loads frame from video
		GetFrame(&capture, &ICur, &ICur_raw, isgrayscale);
		
        // Sets mouse callback fct for template selection
		cv::imshow("Current Image", ICur);        
        cv::setMouseCallback("Current Image", OnMouse, coords);
		cv::waitKey(1);
	}
	
	// If mouse is pressed, Template is selected
	DefineTemplate(&Template, &ICur, coords, isgrayscale);
	
#else
	
	// Loading video
	printf("> Loading video ...\n");
    cv::VideoCapture capture(filename);
	if(!capture.isOpened())
	{
        printf("* Ops! what happened to the video?");
		exit(0);
	}
	printf("> Video loaded...\n");
	
	// Loads frame from video
	capture >> ICur_raw;
	if(ICur_raw.channels() > 1 && isgrayscale == 1)
		cv::cvtColor(ICur_raw, ICur, CV_BGR2GRAY);
	else
		ICur = ICur_raw;

    // Sets mouse callback fct for template selection
    cv::imshow("Current Image", ICur);
    cv::setMouseCallback("Current Image", on_mouse, coords);
	cv::waitKey(0);

	// If mouse is pressed, Template is selected
	DefineTemplate(&Template, &ICur, coords, isgrayscale);

#endif
		
	// Initialize Naya structure
	tracker.Initialize2DOF(size_template_x,
						   size_template_y,
						   n_bins,
						   size_bins,
						   n_max_iters,
						   epsilon,
						   isgrayscale,
						   1);	
	
	// Initialize Naya parameters
	parameters[0] = 1;
	parameters[1] = 0;
	parameters[2] = (float)coords[0];
	parameters[3] = (float)coords[1];

	// Tracking loop
    char key1 = 0;
    while(key1 != 'q' && key1 != 'Q')
    {
		// Grabs image from video
		GetFrame(&capture, &ICur, &ICur_raw, isgrayscale);

		// Run Naya
		tracker.Run2DOF(&ICur, 0, &Template, 0, parameters);
			
		// Display
        key1 = tracker.Display2DOF(&ICur, 1);

		// Clock start/stop
		clock_stop = ((double)cvGetTickCount()-clock_start)/(1000*tick);
		clock_start = (double)cvGetTickCount();
		
		// Terminal
        printf("> Elapsed: %f ms\n", clock_stop);
	}
}
