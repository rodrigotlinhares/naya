
/************************/
/***    naya class   ***/
/**********************/

#include "naya.h"

// This must be first
bool	 myfct (std::pair<int,int> i, std::pair<int,int> j)
{
	return ( (i.second > j.second) );
}

// Initialization

void naya::ComputeActive4DOF(cv::Mat *Input, cv::Mat *MaskInput, int *active_r, int *active_g, int *active_b) 
{
	int n_dof = 4;

	this->Template = Input;
	if(MaskInput != 0)
		this->Mask_template = MaskInput;
	else
		Mask_template = &Mask;

	// Computing image gradient
	WarpGrad(Template);

	// Mounting jacobians
	FastJacobian4DOF(isgrayscale);
	
	// Sort vectors containing gradients
	for(int i=0; i<n_dof; i++)
	{
		std::sort(pair_r[i].begin(), pair_r[i].end(), myfct);
		std::sort(pair_g[i].begin(), pair_g[i].end(), myfct);
		std::sort(pair_b[i].begin(), pair_b[i].end(), myfct);
	}

	// Compute list of active pixels
	int counter_r[4] = {0,0,0,0}, counter_g[4] = {0,0,0,0}, counter_b[4] = {0,0,0,0};
	int n_in = 0, i = 0;
	memset(visited_r, 0, size_template_x*size_template_y);

	// Filling up list of active red pixels
	while(n_in < n_active_pixels)
	{
		if(!visited_r[pair_r[i][counter_r[i]].first])
		{
			visited_r[pair_r[i][counter_r[i]].first] = 1;
			active_r[n_in] = pair_r[i][counter_r[i]].first;

			n_in++;
			i<n_dof-1 ? i++ : i=0;
		}
		else
		{
			counter_r[i] ++;
		}
	}
	
	if(!isgrayscale)
	{
		// Filling up list of active green pixels
		n_in = 0;
		i = 0;
		memset(visited_g, 0, size_template_x*size_template_y);

		while(n_in < n_active_pixels)
		{
			if(!visited_g[pair_g[i][counter_g[i]].first])
			{
				visited_g[pair_g[i][counter_g[i]].first] = 1;
				active_g[n_in] = pair_g[i][counter_g[i]].first;
				n_in++;
				i<n_dof-1 ? i++ : i=0;
			}
			else
			{
				counter_g[i] ++;
			}
		}

		// Filling up list of active blue pixels
		n_in = 0;
		i = 0;
		memset(visited_b, 0, size_template_x*size_template_y);

		while(n_in < n_active_pixels)
		{
			if(!visited_b[pair_b[i][counter_b[i]].first])
			{
				visited_b[pair_b[i][counter_b[i]].first] = 1;
				active_b[n_in] = pair_b[i][counter_b[i]].first;
				n_in++;
				i<n_dof-1 ? i++ : i=0;
			}
			else
			{
				counter_b[i] ++;
			}
		}
	}

	// Display active pixels
	//cv::Mat copyInput;
	//copyInput = Input->clone();
	//copyInput.setTo(0);

	//if(isgrayscale)
	//{
	//	for(int k=0; k<n_active_pixels;k++)
	//	{
	//		// Active pixel positions
	//		int	i1 = cvFloor((float)active_r[k]/size_template_x);
	//		int j1 = active_r[k] - i1*size_template_x;

	//		copyInput.at<uchar>(i1,j1) = Input->at<uchar>(i1,j1);
	//	}
	//}
	//else
	//{
	//	for(int k=0; k<n_active_pixels;k++)
	//	{
	//		// Active pixel positions
	//		int	i1 = cvFloor((float)active_r[k]/size_template_x);
	//		int j1 = active_r[k] - i1*size_template_x;

	//		copyInput.ptr<uchar>(i1)[3*j1+2] = Input->ptr<uchar>(i1)[3*j1+2];
	//		
	//			i1 = cvFloor((float)active_g[k]/size_template_x);
	//		 j1 = active_g[k] - i1*size_template_x;

	//		copyInput.ptr<uchar>(i1)[3*j1+1] = Input->ptr<uchar>(i1)[3*j1+1];
	//	}
	//}

	//cv::imshow("ACTIVEPXL",copyInput);
	//cv::waitKey(0);
}

void naya::ComputeActive8DOF(cv::Mat *Input, cv::Mat *MaskInput, int *active_r, int *active_g, int *active_b) 
{
	int n_dof = 8;

	this->Template = Input;
	if(MaskInput != 0)
		this->Mask_template = MaskInput;
	else
		Mask_template = &Mask;

	// Computing image gradient
	WarpGrad(Template);

	// Mounting jacobians
	FastJacobian8DOF(isgrayscale);
	
	// Sort vectors containing gradients
	for(int i=0; i<n_dof; i++)
	{
		std::sort(pair_r[i].begin(), pair_r[i].end(), myfct);
		std::sort(pair_g[i].begin(), pair_g[i].end(), myfct);
		std::sort(pair_b[i].begin(), pair_b[i].end(), myfct);
	}

	// Compute list of active pixels
	int counter_r[8] = {0,0,0,0,0,0,0,0}, counter_g[8] = {0,0,0,0,0,0,0,0}, counter_b[8] = {0,0,0,0,0,0,0,0};
	int n_in = 0, i = 0;
	memset(visited_r, 0, size_template_x*size_template_y);

	// Filling up list of active red pixels
	while(n_in < n_active_pixels)
	{
		if(!visited_r[pair_r[i][counter_r[i]].first])
		{
			visited_r[pair_r[i][counter_r[i]].first] = 1;
			active_r[n_in] = pair_r[i][counter_r[i]].first;

			n_in++;
			i<n_dof-1 ? i++ : i=0;
		}
		else
		{
			counter_r[i] ++;
		}
	}
	
	if(!isgrayscale)
	{
		// Filling up list of active green pixels
		n_in = 0;
		i = 0;
		memset(visited_g, 0, size_template_x*size_template_y);

		while(n_in < n_active_pixels)
		{
			if(!visited_g[pair_g[i][counter_g[i]].first])
			{
				visited_g[pair_g[i][counter_g[i]].first] = 1;
				active_g[n_in] = pair_g[i][counter_g[i]].first;
				n_in++;
				i<n_dof-1 ? i++ : i=0;
			}
			else
			{
				counter_g[i] ++;
			}
		}

		// Filling up list of active blue pixels
		n_in = 0;
		i = 0;
		memset(visited_b, 0, size_template_x*size_template_y);

		while(n_in < n_active_pixels)
		{
			if(!visited_b[pair_b[i][counter_b[i]].first])
			{
				visited_b[pair_b[i][counter_b[i]].first] = 1;
				active_b[n_in] = pair_b[i][counter_b[i]].first;
				n_in++;
				i<n_dof-1 ? i++ : i=0;
			}
			else
			{
				counter_b[i] ++;
			}
		}
	}

	// Display active pixels
	//cv::Mat copyInput;
	//copyInput = Input->clone();
	//copyInput.setTo(0);

	//if(isgrayscale)
	//{
	//	for(int k=0; k<n_active_pixels;k++)
	//	{
	//		// Active pixel positions
	//		int	i1 = cvFloor((float)active_r[k]/size_template_x);
	//		int j1 = active_r[k] - i1*size_template_x;

	//		copyInput.at<uchar>(i1,j1) = Input->at<uchar>(i1,j1);
	//	}
	//}
	//else
	//{
	//	for(int k=0; k<n_active_pixels;k++)
	//	{
	//		// Active pixel positions
	//		int	i1 = cvFloor((float)active_r[k]/size_template_x);
	//		int j1 = active_r[k] - i1*size_template_x;

	//		copyInput.ptr<uchar>(i1)[3*j1+2] = Input->ptr<uchar>(i1)[3*j1+2];
	//		
	//			i1 = cvFloor((float)active_g[k]/size_template_x);
	//		 j1 = active_g[k] - i1*size_template_x;

	//		copyInput.ptr<uchar>(i1)[3*j1+1] = Input->ptr<uchar>(i1)[3*j1+1];
	//	}
	//}

	//cv::imshow("ACTIVEPXL",copyInput);
	//cv::waitKey(0);
}


void	naya::FastJacobian4DOF(int isgrayscale)
{  
	int offx = cvCeil((double)size_template_x/2);
	int offy = cvCeil((double)size_template_y/2);

	// Mounting matrix 
	for(int i=0;i<size_template_y;i++)
	{
		for(int j=0;j<size_template_x;j++)
		{
			if(!isgrayscale)
			{
				// GREEN
				pair_g[0][j+size_template_x*i].first = j+size_template_x*i;
				pair_g[1][j+size_template_x*i].first = j+size_template_x*i;
				pair_g[2][j+size_template_x*i].first = j+size_template_x*i;
				pair_g[3][j+size_template_x*i].first = j+size_template_x*i;

				// BLUE
				pair_b[0][j+size_template_x*i].first = j+size_template_x*i;
				pair_b[1][j+size_template_x*i].first = j+size_template_x*i;
				pair_b[2][j+size_template_x*i].first = j+size_template_x*i;
				pair_b[3][j+size_template_x*i].first = j+size_template_x*i;
			}

			// RED
			pair_r[0][j+size_template_x*i].first = j+size_template_x*i;
			pair_r[1][j+size_template_x*i].first = j+size_template_x*i;
			pair_r[2][j+size_template_x*i].first = j+size_template_x*i;
			pair_r[3][j+size_template_x*i].first = j+size_template_x*i;

			if(Mask_template->at<uchar>(i,j) != 0)
			{
				if(!isgrayscale)
				{
					// RED
					pair_r[0][j+size_template_x*i].second = (int)std::abs(8<<((j+1-offx)*(int)gradx.ptr<float>(i)[3*j] + (i+1-offy)*(int)grady.ptr<float>(i)[3*j]));
					pair_r[1][j+size_template_x*i].second = (int)std::abs(8<<(-(i+1-offy)*(int)gradx.ptr<float>(i)[3*j] + (j+1-offx)*(int)grady.ptr<float>(i)[3*j]));
					pair_r[2][j+size_template_x*i].second = (int)std::abs(8<<(int)gradx.ptr<float>(i)[3*j]);
					pair_r[3][j+size_template_x*i].second = (int)std::abs(8<<(int)grady.ptr<float>(i)[3*j]);
					
					// GREEN
					pair_g[0][j+size_template_x*i].second = (int)std::abs(8<<((j+1-offx)*(int)gradx.ptr<float>(i)[3*j+1] + (i+1-offy)*(int)grady.ptr<float>(i)[3*j+1]));
					pair_g[1][j+size_template_x*i].second = (int)std::abs(8<<(-(i+1-offy)*(int)gradx.ptr<float>(i)[3*j+1] + (j+1-offx)*(int)grady.ptr<float>(i)[3*j+1]));
					pair_g[2][j+size_template_x*i].second = (int)std::abs(8<<(int)gradx.ptr<float>(i)[3*j+1]);
					pair_g[3][j+size_template_x*i].second = (int)std::abs(8<<(int)grady.ptr<float>(i)[3*j+1]);
					
					// BLUE
					pair_b[0][j+size_template_x*i].second = (int)std::abs(8<<((j+1-offx)*(int)gradx.ptr<float>(i)[3*j+2] + (i+1-offy)*(int)grady.ptr<float>(i)[3*j+2]));
					pair_b[1][j+size_template_x*i].second = (int)std::abs(8<<(-(i+1-offy)*(int)gradx.ptr<float>(i)[3*j+2] + (j+1-offx)*(int)grady.ptr<float>(i)[3*j+2]));
					pair_b[2][j+size_template_x*i].second = (int)std::abs(8<<(int)gradx.ptr<float>(i)[3*j+2]);
					pair_b[3][j+size_template_x*i].second = (int)std::abs(8<<(int)grady.ptr<float>(i)[3*j+2]);
				}
				else
				{
					// RED
					pair_r[0][j+size_template_x*i].second = (int)std::abs(8<<((j+1-offx)*(int)gradx.at<float>(i,j) + (i+1-offy)*(int)grady.at<float>(i,j)));
					pair_r[1][j+size_template_x*i].second = (int)std::abs(8<<(-(i+1-offy)*(int)gradx.at<float>(i,j) + (j+1-offx)*(int)grady.at<float>(i,j)));
					pair_r[2][j+size_template_x*i].second = (int)std::abs((int)gradx.at<float>(i,j));
					pair_r[3][j+size_template_x*i].second = (int)std::abs((int)grady.at<float>(i,j));
				}
			}
			else
			{
				if(!isgrayscale)
				{
					// BLUE
					pair_b[0][j+size_template_x*i].second = 0;
					pair_b[1][j+size_template_x*i].second = 0;
					pair_b[2][j+size_template_x*i].second = 0;
					pair_b[3][j+size_template_x*i].second = 0;

					// GREEN
					pair_g[0][j+size_template_x*i].second = 0;
					pair_g[1][j+size_template_x*i].second = 0;
					pair_g[2][j+size_template_x*i].second = 0;
					pair_g[3][j+size_template_x*i].second = 0;
				}

				// RED
				pair_r[0][j+size_template_x*i].second = 0;
				pair_r[1][j+size_template_x*i].second = 0;
				pair_r[2][j+size_template_x*i].second = 0;
				pair_r[3][j+size_template_x*i].second = 0;
			}
		}
	}	
}

void	naya::FastJacobian8DOF(int isgrayscale)
{  
	int offx = cvCeil((double)size_template_x/2);
	int offy = cvCeil((double)size_template_y/2);

	// Mounting matrix 
	for(int i=0;i<size_template_y;i++)
	{
		for(int j=0;j<size_template_x;j++)
		{
			// RED
			pair_r[0][j+size_template_x*i].first = j+size_template_x*i;
			pair_r[1][j+size_template_x*i].first = j+size_template_x*i;
			pair_r[2][j+size_template_x*i].first = j+size_template_x*i;
			pair_r[3][j+size_template_x*i].first = j+size_template_x*i;
			pair_r[4][j+size_template_x*i].first = j+size_template_x*i;
			pair_r[5][j+size_template_x*i].first = j+size_template_x*i;
			pair_r[6][j+size_template_x*i].first = j+size_template_x*i;
			pair_r[7][j+size_template_x*i].first = j+size_template_x*i;

			if(!isgrayscale)
			{
				// GREEN
				pair_g[0][j+size_template_x*i].first = j+size_template_x*i;
				pair_g[1][j+size_template_x*i].first = j+size_template_x*i;
				pair_g[2][j+size_template_x*i].first = j+size_template_x*i;
				pair_g[3][j+size_template_x*i].first = j+size_template_x*i;
				pair_g[4][j+size_template_x*i].first = j+size_template_x*i;
				pair_g[5][j+size_template_x*i].first = j+size_template_x*i;
				pair_g[6][j+size_template_x*i].first = j+size_template_x*i;
				pair_g[7][j+size_template_x*i].first = j+size_template_x*i;
				

				// BLUE
				pair_b[0][j+size_template_x*i].first = j+size_template_x*i;
				pair_b[1][j+size_template_x*i].first = j+size_template_x*i;
				pair_b[2][j+size_template_x*i].first = j+size_template_x*i;
				pair_b[3][j+size_template_x*i].first = j+size_template_x*i;
				pair_b[4][j+size_template_x*i].first = j+size_template_x*i;
				pair_b[5][j+size_template_x*i].first = j+size_template_x*i;
				pair_b[6][j+size_template_x*i].first = j+size_template_x*i;
				pair_b[7][j+size_template_x*i].first = j+size_template_x*i;				
			}
			
			if(Mask_template->at<uchar>(i,j) != 0)
			{
				if(!isgrayscale)
				{			
					int gradxint = 8<<((int)gradx.ptr<float>(i)[3*j+2]);
					int gradyint = 8<<((int)grady.ptr<float>(i)[3*j+2]);
					int temp = -j*gradxint-i*gradyint;

					// RED
					pair_r[0][j+size_template_x*i].second = (int)std::abs(gradxint);
					pair_r[1][j+size_template_x*i].second = (int)std::abs(gradyint);
					pair_r[2][j+size_template_x*i].second = (int)std::abs(i*gradxint);
					pair_r[3][j+size_template_x*i].second = (int)std::abs(j*gradyint);
					pair_r[4][j+size_template_x*i].second = (int)std::abs(-i*gradyint+j*gradxint);
					pair_r[5][j+size_template_x*i].second = (int)std::abs(temp - i*gradyint);
					pair_r[6][j+size_template_x*i].second = (int)std::abs(j*temp);
					pair_r[7][j+size_template_x*i].second = (int)std::abs(i*temp);
					
					// GREEN
					gradxint = 8<<((int)gradx.ptr<float>(i)[3*j+1]);
					gradyint = 8<<((int)grady.ptr<float>(i)[3*j+1]);
					temp = -j*gradxint-i*gradyint;
					
					pair_g[0][j+size_template_x*i].second = (int)std::abs(gradxint);
					pair_g[1][j+size_template_x*i].second = (int)std::abs(gradyint);
					pair_g[2][j+size_template_x*i].second = (int)std::abs(i*gradxint);
					pair_g[3][j+size_template_x*i].second = (int)std::abs(j*gradyint);
					pair_g[4][j+size_template_x*i].second = (int)std::abs(-i*gradyint+j*gradxint);
					pair_g[5][j+size_template_x*i].second = (int)std::abs(temp - i*gradyint);
					pair_g[6][j+size_template_x*i].second = (int)std::abs(j*temp);
					pair_g[7][j+size_template_x*i].second = (int)std::abs(i*temp);
					
					// BLUE
					gradxint = 8<<((int)gradx.ptr<float>(i)[3*j]);
					gradyint = 8<<((int)grady.ptr<float>(i)[3*j]);
					temp = -j*gradxint-i*gradyint;
					
					pair_b[0][j+size_template_x*i].second = (int)std::abs(gradxint);
					pair_b[1][j+size_template_x*i].second = (int)std::abs(gradyint);
					pair_b[2][j+size_template_x*i].second = (int)std::abs(i*gradxint);
					pair_b[3][j+size_template_x*i].second = (int)std::abs(j*gradyint);
					pair_b[4][j+size_template_x*i].second = (int)std::abs(-i*gradyint+j*gradxint);
					pair_b[5][j+size_template_x*i].second = (int)std::abs(temp - i*gradyint);
					pair_b[6][j+size_template_x*i].second = (int)std::abs(j*temp);
					pair_b[7][j+size_template_x*i].second = (int)std::abs(i*temp);
				}
				else
				{	
					int gradxint = 8<<((int)gradx.at<float>(i,j));
					int gradyint = 8<<((int)grady.at<float>(i,j));
					int temp = -j*gradxint-i*gradyint;
					
					pair_r[0][j+size_template_x*i].second = (int)std::abs(gradxint);
					pair_r[1][j+size_template_x*i].second = (int)std::abs(gradyint);
					pair_r[2][j+size_template_x*i].second = (int)std::abs(i*gradxint);
					pair_r[3][j+size_template_x*i].second = (int)std::abs(j*gradyint);
					pair_r[4][j+size_template_x*i].second = (int)std::abs(-i*gradyint+j*gradxint);
					pair_r[5][j+size_template_x*i].second = (int)std::abs(temp - i*gradyint);
					pair_r[6][j+size_template_x*i].second = (int)std::abs(j*temp);
					pair_r[7][j+size_template_x*i].second = (int)std::abs(i*temp);
				}
			}
			else
			{
				// RED
				pair_r[0][j+size_template_x*i].second = 0;
				pair_r[1][j+size_template_x*i].second = 0;
				pair_r[2][j+size_template_x*i].second = 0;
				pair_r[3][j+size_template_x*i].second = 0;
				pair_r[4][j+size_template_x*i].second = 0;
				pair_r[5][j+size_template_x*i].second = 0;
				pair_r[6][j+size_template_x*i].second = 0;
				pair_r[7][j+size_template_x*i].second = 0;

				if(!isgrayscale)
				{
					// BLUE
					pair_b[0][j+size_template_x*i].second = 0;
					pair_b[1][j+size_template_x*i].second = 0;
					pair_b[2][j+size_template_x*i].second = 0;
					pair_b[3][j+size_template_x*i].second = 0;
					pair_b[4][j+size_template_x*i].second = 0;
					pair_b[5][j+size_template_x*i].second = 0;
					pair_b[6][j+size_template_x*i].second = 0;
					pair_b[7][j+size_template_x*i].second = 0;

					// GREEN
					pair_g[0][j+size_template_x*i].second = 0;
					pair_g[1][j+size_template_x*i].second = 0;
					pair_g[2][j+size_template_x*i].second = 0;
					pair_g[3][j+size_template_x*i].second = 0;
					pair_g[4][j+size_template_x*i].second = 0;
					pair_g[5][j+size_template_x*i].second = 0;
					pair_g[6][j+size_template_x*i].second = 0;
					pair_g[7][j+size_template_x*i].second = 0;
				}
			}
		}
	}	
}
