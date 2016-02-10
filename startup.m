addpath('external/genpath_exclude');
dir='.'         ;try,addpath(genpath_exclude(dir,{'.svn','doc','external'})),catch,warning(['Directory ',dir,' not found.']),end
%dir='f'         ;try,addpath(genpath_exclude(dir,{'.svn'})),catch,warning(['Directory ',dir,' not found.']),end
%dir='actuators' ;try,addpath(genpath_exclude(dir,{'.svn'})),catch,warning(['Directory ',dir,' not found.']),end
%dir='dynamics'  ;try,addpath(genpath_exclude(dir,{'.svn'})),catch,warning(['Directory ',dir,' not found.']),end
%dir='examples'  ;try,addpath(genpath_exclude(dir,{'.svn'})),catch,warning(['Directory ',dir,' not found.']),end
%dir='kinematics';try,addpath(genpath_exclude(dir,{'.svn'})),catch,warning(['Directory ',dir,' not found.']),end
%dir='models'    ;try,addpath(genpath_exclude(dir,{'.svn'})),catch,warning(['Directory ',dir,' not found.']),end
%dir='simulate'  ;try,addpath(genpath_exclude(dir,{'.svn'})),catch,warning(['Directory ',dir,' not found.']),end
