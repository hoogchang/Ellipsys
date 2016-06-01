  function PrintTo2BiquadFHeaderFile(fid, sos, name, sos2, name2, sos3, name3, T, comment)
  
  % Print to the filter definition for FLOATING POINT header file.
  %
  %   PrintToBiquadFHeaderFile(fid, sos, name)
  %
  %--- fid      - File indentity
  %--- sos      - Scaled second order sections, from "tf2sos"
  %--- name     - Name to be given to the array of biquad structures, and
  %                  associated with the number of sections.
  %--- T        - Sample period in seconds
  %--- comment  - comment added at top of header
  
%---structure form of cascade

fprintf(fid,'//---%s\n', comment);
[ns,m]=size(sos);
fprintf(fid,'    int         %s_ns = %d;     // number of sections\n',name,ns);
% fprintf(fid,'    uint32_t    timeoutValue = %d;  // time interval - us; f_s = %g Hz\n',T*1e6,1/T);
fprintf(fid,'    static\tstruct\tbiquad %s[]={ 	 // define the array of floating point biquads\n',name);
for i=1:ns-1
    fprintf(fid,'        {');
    for j=[1,2,3,4,5,6]
        fprintf(fid,'%e, ',sos(i,j));
    end
    fprintf(fid,'0, 0, 0, 0, 0},\n');
end
    fprintf(fid,'        {');
    for j=[1,2,3,4,5,6]
        fprintf(fid,'%e, ',sos(ns,j));
    end
    fprintf(fid,'0, 0, 0, 0, 0}\n        };\n');
    
[ns2,m2]=size(sos2);
fprintf(fid,'    int         %s_ns = %d;     // number of sections\n',name2,ns2);
fprintf(fid,'    uint32_t    timeoutValue = %d;  // time interval - us; f_s = %g Hz\n',T*1e6,1/T);
fprintf(fid,'    static\tstruct\tbiquad %s[]={ 	 // define the array of floating point biquads\n',name2);
for i=1:ns2-1
    fprintf(fid,'        {');
    for j=[1,2,3,4,5,6]
        fprintf(fid,'%e, ',sos2(i,j));
    end
    fprintf(fid,'0, 0, 0, 0, 0},\n');
end
    fprintf(fid,'        {');
    for j=[1,2,3,4,5,6]
        fprintf(fid,'%e, ',sos2(ns2,j));
    end
    fprintf(fid,'0, 0, 0, 0, 0}\n        };\n');

[ns3,m3]=size(sos3);
fprintf(fid,'    int         %s_ns = %d;     // number of sections\n',name3,ns3);
% fprintf(fid,'    uint32_t    timeoutValue = %d;  // time interval - us; f_s = %g Hz\n',T*1e6,1/T);
fprintf(fid,'    static\tstruct\tbiquad %s[]={ 	 // define the array of floating point biquads\n',name3);
for i=1:ns3-1
    fprintf(fid,'        {');
    for j=[1,2,3,4,5,6]
        fprintf(fid,'%e, ',sos3(i,j));
    end
    fprintf(fid,'0, 0, 0, 0, 0},\n');
end
    fprintf(fid,'        {');
    for j=[1,2,3,4,5,6]
        fprintf(fid,'%e, ',sos3(ns3,j));
    end
    fprintf(fid,'0, 0, 0, 0, 0}\n        };\n');
