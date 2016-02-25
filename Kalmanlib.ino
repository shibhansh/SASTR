#include <stdio.h>
#include <string.h>
void transpose(double *f,int row1, int col1,double *g, int row2, int col2){
  if(row1!=col2||col1!=row2){
    printf("wrong input\n");
    return;
  }
  int i,j;
  for(i=0;i<row1;i++){
    for(j=0;j<col1;j++){
      *(g+i*col2+j) = *(f+j*col1+i) ;
    }
  }
  return;
}
void inverse(double *f,int row1, int col1,double *g, int row2, int col2){
  if(row1!=col1||row2!=col2){
    printf("one of the matrices is not a square matrix\n");
    return;
  }
  if(row2!=row1){
    printf("matrices of diffrent sizes\n");
    return;
  }
  if(row1!=1){
    printf("function is not capable of calculating inverse for matrices of size more than 1\n");
  }
  if(*(f)!=0)
    *(g)=1/(*(f));
  else
    printf("singular matrix inverse does not exist\n");
  return;
}
//this function will subtract matrix g from matrix f
void subtract(double *f,int row1, int col1,double *g, int row2, int col2,double *h,int row,int col){
  if(row1!=row2||row!=row2){
    printf("no. of rows mismatch\n");
    return;
  }
  if(col!=col1||col!=col2){
    printf("no. of columns mismatch\n");
    return;
  }
  int i,j;
  for(i=0;i<row;i++){
    for(j=0;j<col;j++){
      *(h+i*col+j) = *(f+i*col+j) - *(g+i*col+j) ;
    }
  }
  return;
}
void summation(double *f,int row1, int col1,double *g, int row2, int col2,double *h,int row,int col){
  if(row1!=row2||row!=row2){
    printf("no. of rows mismatch\n");
    return;
  }
  if(col!=col1||col!=col2){
    printf("no. of columns mismatch\n");
    return;
  }
  int i,j;
  for(i=0;i<row;i++){
    for(j=0;j<col;j++){
      *(h+i*col+j) = *(f+i*col+j) + *(g+i*col+j) ;
    }
  }
  return;
}
//this product function expects inputs like matrix[0] and not matrix
void product(double *f,int row1, int col1,double *g, int row2, int col2,double *h,int row,int col){
  if(col1!=row2){
    printf("matrix multiplication is not allowed \n");
    return;
  }
  if(row1!=row||col2!=col){
    printf("the dimensions of the output matrix are incorrect\n");
    return;
  }
  int i,j,k;
  float sum=0;
  for(i=0;i<row;i++){
    for(j=0;j<col;j++){
      sum=0;
      for(k=0;k<col1;k++){
        sum+=(*(f+i*col1+k))*(*(g+k*col2+j));
      }
      *(h+i*col+j)=sum;
    }
  }
  return;
}
void covariance(double *f, int row,double *g){
  int i,j;
  for(i=0;i<row;i++){
    for(j=0;j<row;j++){
      *(g+i*row+j)=(*(f+i+j))*(*(f+i+j));
    }
  }
  return;
}
