#ifndef naya_H
#define naya_H

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <cstdio>

class naya
{
	// Pixel selection part
	
public:

	void	ComputeActive4DOF(cv::Mat *Input, cv::Mat *MaskInput, int *active_r, int *active_g, int *active_b);
	
	void	ComputeActive8DOF(cv::Mat *Input, cv::Mat *MaskInput, int *active_r, int *active_g, int *active_b);

private:
	
	void	FastJacobian4DOF(int isgrayscale);

	void	FastJacobian8DOF(int isgrayscale);


	// Tracking part

public:
	
    void	Initialize2DOF(int size_template_x,
						   int size_template_y,
						   int n_bins,
						   int size_bins,
						   int n_max_iters,
						   float epsilon,
						   int isgrayscale,
						   int interp);   

    void	Initialize4DOF(int size_template_x,
						   int size_template_y,
						   int n_active_pixels,
						   int n_bins,
						   int size_bins,
						   int n_max_iters,
						   float epsilon,
						   int isgrayscale,
						   int interp);

    void    Initialize6DOF(int size_template_x,
                           int size_template_y,
                           int n_bins,
                           int size_bins,
                           int n_max_iters,
                           float epsilon,
                           int isgrayscale,
                           int interp);
	
	void	Initialize8DOF(int size_template_x,
						   int size_template_y,
						   int n_active_pixels,
						   int n_bins,
						   int size_bins,
						   int n_max_iters,
						   float epsilon,
						   int isgrayscale,
						   int interp);
	
	void	InitializeTPS(int size_template_x,
						  int size_template_y,
						  int n_active_pixels,
						  int n_ctrl_pts_x,
						  int n_ctrl_pts_y,
						  float lambda,
						  int n_bins,
						  int size_bins,
						  int n_max_iters,
						  float epsilon,
						  int isgrayscale,
						  int interp);

    void    InitializeCompositional6DOF(int size_template_x,
                                        int size_template_y,
                                        int n_bins,
                                        int size_bins,
                                        int n_max_iters,
                                        float epsilon,
                                        int isgrayscale,
                                        int interp);

	int 	Run2DOF(cv::Mat *ICur, 
					cv::Mat *Mask_roi,
					cv::Mat *Template,
					cv::Mat *Mask_template,
					float *parameters);

	int 	Run4DOF(cv::Mat *ICur, 
					cv::Mat *Mask_roi,
					cv::Mat *Template,
					cv::Mat *Mask_template,
					float *parameters,
				   int *active_pixels_r,
				   int *active_pixels_g,
				   int *active_pixels_b);
	
    int     Run6DOF(cv::Mat *ICur, 
                    cv::Mat *Mask_roi,
                    cv::Mat *Template,
                    cv::Mat *Mask_template,
                    float *parameters);
	
	int 	Run8DOF(cv::Mat *ICur, 
					cv::Mat *Mask_roi,
					cv::Mat *Template,
					cv::Mat *Mask_template,
					float *parameters,
					int *active_pixels_r,
					int *active_pixels_g,
					int *active_pixels_b);	

	int 	RunTPS(cv::Mat *ICur, 
				   cv::Mat *Mask_roi,
				   cv::Mat *Template,
				   cv::Mat *Mask_template,
				   float *parameters,
				   int *active_pixels_r,
				   int *active_pixels_g,
				   int *active_pixels_b);	
	
	
    int 	Display2DOF(cv::Mat *ICur, int delay);
	
    int     Display4DOF(cv::Mat *ICur, int delay);
	
    int     Display6DOF(cv::Mat *ICur, int delay);

    int 	Display8DOF(cv::Mat *ICur, int delay);

    int 	DisplayTPS(cv::Mat *ICur, int delay);
	
	
	void	InitTPSParameters(float *parameters, int *coords);
	
	void	ResetIlluminationParam3DOFxi(float *illum_param);


	void	ResetExpected();

    float	ComputeTrackingConfidenceSCV();
		

	bool	CheckConsistency2DOF();

	bool	CheckConsistency4DOF();


	cv::Mat* GetPtrMapx();

	cv::Mat* GetPtrMapy();
	
private:
	
	void	Warp2DOF();

    void	Warp4DOF();

    void    Warp6DOF();
	
    void	Warp8DOF();

    void	WarpTPS();
	
	
	void	WarpGrad();

	void	WarpGradi();

	void	WarpGrad(cv::Mat *Input);


    int		ComputeJointHistogramGray();
	
    int		ComputeJointHistogramColor();


	void	ComputeExpectedImg();
	
	
    void	MountJacobian2DOFGray();

    void	MountJacobian2DOFColor();


    void	MountJacobian4DOFGray();

    void	MountJacobian4DOFColor();


    void	MountJacobian6DOFGray();

    void	MountJacobian6DOFColor();


    void	MountJacobian8DOFGray();

    void	MountJacobian8DOFColor();
	
	
	void	MountJacobianTPSGray();

	void	MountJacobianTPSColor();
	
	
    int		Update2DOF();

    int		Update4DOF();

    int     Update6DOF();
	
    int		Update8DOF();

	int		UpdateTPS();


	void	OcclusionMap();
	
	
	double	Max(cv::Mat *M);
		
	void	MyExpm(cv::Mat *input);

	
	void	DefineCtrlPts();

	void	TPSPrecomputations();

	float	Tps(float r);

	float	Norm(float x, float y);
	
	void	ParameterIO(int isinput);

	void	GetPos(int x, int y, float *wx, float *wy);	


	void	TPSPrecomputationsIllum();

	void	DefineCtrlPtsIllum();

	void	ParameterIOIllum(int isinput);

	void	NonRigidCompensation();


private:

	// Are we using masks?
	bool	using_masks;

	// naya parameters
	int		size_template_x,
			size_template_y,
			n_bins,
			size_bins,
			n_active_pixels,
			n_max_iters,
			interp,
			isgrayscale;

	float	epsilon;
	
	// List of active pixels	
	bool	 *visited_r,
			 *visited_g,
			 *visited_b;

	int		*active_pixels_r,
		    *active_pixels_g,
		    *active_pixels_b;

	int		*std_pixel_list;
	
    std::vector<std::pair<int, int> > pair_r[8],
									 pair_g[8],
									 pair_b[8];

	// SCV aux
	float	*correction,
			*p_joint,
			*expected;
	
	// naya parameters
	float	*parameters,
			 angle_3DOF;
	
	// Input images and masks for naya
	cv::Mat	*ICur,
			*Template,
			*Mask_template,
			*Mask_roi;

	// naya core stuff
	cv::Mat dummy_mapx,
			dummy_mapy,
			SD,
			Hess,
			delta,
			dif,
			gradx,
			grady,
			gradx_tmplt,
			grady_tmplt;

	// 8dof update 
	cv::Mat update_auxA,
			update_auxH,
			aux1,
			aux2,
			aux3,
			aux4,
			aux5;

	// TPS parameters
	int		n_ctrl_pts_x,
		    n_ctrl_pts_y,
			total_n_ctrl_pts;

	int		*ctrl_pts_x,
			*ctrl_pts_y;

	float	lambda;

	cv::Mat	ctrl_pts_x_w,
			ctrl_pts_y_w,
			list_ctrl_pts;

	cv::Mat MKinv,
			Ks,
			Ksw;
	
    cv::Mat weights;

public:

	// Current image and mask
	cv::Mat current_warp,
			compensated_warp,
			Template_comp,
			Mask;

	// naya stats
    int		iters;
};

#endif
