% Function will take a colour image as input  and will return a grayscale image 
function [gray_img] = colouredToGray(img)
       % Extract Red colour component to R, Green colour component to G 
       % and Blue colour component to  B  
       R=img(:, :, 1);
       G=img(:, :, 2);
       B=img(:, :, 3);

      % Getting number of rows in M and number of column in N of RGB image matrix 
      % ~ is used to ignore dimension of RGB image
      % as size(img) function will return row, column and dimension of the RGB image
      [M, N, ~]=size(img);

     % creating a new 2-d matrix 'gray_img' of size M*N of 'uint8' data type with all 
     % elements  as zero 
     gray_img=zeros(M, N, 'uint8');

     % calculating grayscale values by forming a weighted sum of the R, G, and B components
     % for each pixel
     for i=1:M
         for j=1:N
           gray_img(i, j)=(R(i, j)*0.2989)+(G(i, j)*0.5870)+(B(i, j)*0.114);
         end
     end
end