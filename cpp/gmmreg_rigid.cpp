#include "gmmreg_rigid.h"

#include <iostream>

#include <vnl/algo/vnl_lbfgsb.h>
#include <vnl/vnl_matrix.h>
#include <vnl/vnl_vector.h>

#include "utils/io_utils.h"
#include "utils/rotation_utils.h"

namespace gmmreg {

const float kPi = 3.1415926f;

void ConvertParamToRotationAndTranslation(const vnl_vector<double>& param,
                                          const int d,
                                          vnl_matrix<double>& rotation,
                                          vnl_matrix<double>& translation) {
  rotation.set_size(d, d);
  translation.set_size(1, d);
  if (d == 2) {
    translation[0][0] = param[0];
    translation[0][1] = param[1];
    double theta = param[2];
    rotation[0][0] = cos(theta);
    rotation[0][1] = -sin(theta);
    rotation[1][0] = sin(theta);
    rotation[1][1] = cos(theta);
  } else if (d == 3) {
    vnl_vector<double> q;
    q.set_size(4);
    for (int i = 0; i < 4; ++i) {
      q[i] = param[i];
    }
    Quaternion2Rotation<double>(q, rotation);
    translation[0][0] = param[4];
    translation[0][1] = param[5];
    translation[0][2] = param[6];
  }
}

void ConvertRigidParamToMatrix(const vnl_vector<double>& param, const int d,
                               vnl_matrix<double>* matrix) {
  vnl_matrix<double> translation;
  vnl_matrix<double> rotation;
  matrix->set_size(d + 1, d + 1);
  matrix->set_row(d, 0.0);
  (*matrix)[d][d] = 1.0;
  ConvertParamToRotationAndTranslation(param, d, rotation, translation);
  matrix->update(rotation, 0, 0);
  matrix->update(translation.transpose(), 0, d);
}

void SetRigidTransformBound(const int d, vnl_lbfgsb* minimizer) {
  // http://public.kitware.com/vxl/doc/release/core/vnl/html/vnl__lbfgsb_8h_source.html
  vnl_vector<long> nbd;
  vnl_vector<double> lower_bound, upper_bound;
  if (d == 2) {
    nbd.set_size(3);  // (delta_x, delta_y, delta_\theta)
    nbd[0] = 0;       // delta_x unconstrained
    nbd[1] = 0;       // delta_y unconstrained
    nbd[2] = 2;       // delta_\theta has both lower and upper bounds
    lower_bound.set_size(3);
    lower_bound.fill(0);
    lower_bound[2] = -1.0 * kPi;
    upper_bound.set_size(3);
    upper_bound.fill(0);
    upper_bound[2] = kPi;
  } 
  else if (d == 3) {
    nbd.set_size(7);  // ((q1*i + q2*j + q3*k + q4), (dx, dy, dz))
    nbd.fill(0);
    for (int i = 0; i < 4; ++i) {
      nbd[i] = 2;  // quaternion part has both lower and upper bounds
    }
    lower_bound.set_size(7);
    lower_bound.fill(0);
    for (int i = 0; i < 4; ++i) {
      lower_bound[i] = -1;
    }
    upper_bound.set_size(7);
    upper_bound.fill(0);
    for (int i = 0; i < 4; ++i) {
      upper_bound[i] = 1;
    }
  }
  minimizer->set_bound_selection(nbd);
  minimizer->set_lower_bound(lower_bound);
  minimizer->set_upper_bound(upper_bound);
}

void RigidRegistration::StartRegistration(vnl_vector<double>& params) {

  //有参构造简单赋值
  // 调用 init_param函数
  vnl_lbfgsb minimizer(*func_);
  // 四元数部分旋转参数的上上下界设置为0-1，平移矩阵部分的参数没有上下界
  SetRigidTransformBound(this->d_, &minimizer);
  //给函数的实属性 通过点云配准实例赋值 把this传过去的过程实际上也是把读取的点运数据读过去了
  func_->SetBase(this);

  for (unsigned int k = 0; k < this->level_; ++k) {
    std::cout<<"####################################"<<std::endl;  
    std::cout<<"第"<<k<<"级别(level)迭代！"<<std::endl;

    //设置 sigmag 给cost函数
    func_->SetScale(this->v_scale_[k]);
    printf("  sigma = %lf\n",k,this->v_scale_[k]);
    //printf("max_func_eval is = %d\n",k,this->v_func_evals_[k]);
    
    //将旋转参数传递给 params 返回出来
    SetParam(params);
    minimizer.set_max_function_evals(this->v_func_evals_[k]);
    minimizer.set_f_tolerance(1e-7);
    minimizer.set_x_tolerance(1e-5);
    // For more options, see
    // https://public.kitware.com/vxl/doc/release/core/vnl/html/vnl__nonlinear__minimizer_8h_source.html
  
    //将params作为x 未知数

    //最关键的一步
    minimizer.minimize(params);
    printf("minimizer.minimize over\n");

    if (minimizer.get_failure_code() < 0) {
      break;
    }

    printf("计算函数值 ...");
    double fxval = func_->f(params);

    std::cout << "目前函数最小值是 " << fxval << std::endl
              << "# 循环迭代次数（每一次循环包含若干次函数计算）: " << minimizer.get_num_iterations() << ", "
              << "# 函数总共求值次数: " << minimizer.get_num_evaluations()
              << std::endl;
    std::cout<<"第"<<k<<"级别(level)迭代结束END！"<<std::endl;
    
  }
  std::cout<<"点云配准结束！"<<std::endl;
}

int RigidRegistration::SetInitParams(const char* f_config) {
  char f_init_rigid[80] = {0};\
  // ""
  GetPrivateProfileString(this->section_, "init_rigid", NULL, f_init_rigid, 80,
                          f_config);
  printf("f_initconfig: %s\n", f_config);
  SetInitRigid(f_init_rigid);
  return 0;
}

int RigidRegistration::SetInitRigid(const char* filename) {
  // TODO(bing-jian): *really* set initial rigid parameter from given file.
  if (this->d_ == 2) {
    param_rigid_.set_size(3);
    param_rigid_.fill(0);
    param_rigid_[0] = 0;
    param_rigid_[1] = 0;
    param_rigid_[2] = 0;
  } else if (this->d_ == 3) {
    param_rigid_.set_size(7);
    param_rigid_.fill(0);
    param_rigid_[3] = 1;  // q = (0, 0, 0, 1) for eye(3)
  }
  return 0;
}

void RigidRegistration::SetParam(vnl_vector<double>& x0) {
  x0 = this->param_rigid_;
}

void RigidRegistration::PerformTransform(const vnl_vector<double>& x) {
  vnl_matrix<double> translation;
  vnl_matrix<double> rotation;
  vnl_matrix<double> ones;
  ones.set_size(this->m_, 1);
  ones.fill(1);
  ConvertParamToRotationAndTranslation(x, this->d_, rotation, translation);
  this->transformed_model_ =
      this->model_ * rotation.transpose() + ones * translation;
  this->param_rigid_ = x;
}

void RigidRegistration::SaveResults(const char* f_config,
                                    const vnl_vector<double>& params) {
  SaveElapsedTime(f_config);
  char f_transformed[256] = {0};
  char f_final_rigid[256] = {0};
  char f_final_matrix[256] = {0};

  GetPrivateProfileString(this->common_section_, "transformed_model", NULL,
                          f_transformed, 256, f_config);
  this->SaveTransformed(f_transformed, params, f_config);

  GetPrivateProfileString(this->common_section_, "final_rigid", NULL,
                          f_final_rigid, 256, f_config);
  SaveVectorToAsciiFile(f_final_rigid, this->param_rigid_);

  vnl_matrix<double> matrix;
  ConvertRigidParamToMatrix(this->param_rigid_, this->d_, &matrix);
  GetPrivateProfileString(this->common_section_, "final_rigid_matrix", NULL,
                          f_final_matrix, 256, f_config);
  SaveMatrixToAsciiFile(f_final_matrix, matrix);
}

void RigidRegistration::PrepareOwnOptions(const char* f_config) {
  this->MultiScaleOptions(f_config);
}

}  // namespace gmmreg
