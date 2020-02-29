% Converts encoder values into angles
function [enc] = convertToEnc(angle)
enc=angle*(4096/(2*pi));
end