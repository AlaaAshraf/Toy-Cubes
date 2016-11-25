function varargout = GUI(varargin)
% GUI MATLAB code for GUI.fig
%      GUI, by itself, creates a new GUI or raises the existing
%      singleton*.
%
%      H = GUI returns the handle to a new GUI or the handle to
%      the existing singleton*.
%
%      GUI('CALLBACK',hObject,eventData,handles,...) calls the local
%      function named CALLBACK in GUI.M with the given input arguments.
%
%      GUI('Property','Value',...) creates a new GUI or raises the
%      existing singleton*.  Starting from the left, property value pairs are
%      applied to the GUI before GUI_OpeningFcn gets called.  An
%      unrecognized property name or invalid value makes property application
%      stop.  All inputs are passed to GUI_OpeningFcn via varargin.
%
%      *See GUI Options on GUIDE's Tools menu.  Choose "GUI allows only one
%      instance to run (singleton)".
%
% See also: GUIDE, GUIDATA, GUIHANDLES

% Edit the above text to modify the response to help GUI

% Last Modified by GUIDE v2.5 19-Nov-2016 15:48:53

% Begin initialization code - DO NOT EDIT
gui_Singleton = 1;
gui_State = struct('gui_Name',       mfilename, ...
                   'gui_Singleton',  gui_Singleton, ...
                   'gui_OpeningFcn', @GUI_OpeningFcn, ...
                   'gui_OutputFcn',  @GUI_OutputFcn, ...
                   'gui_LayoutFcn',  [] , ...
                   'gui_Callback',   []);
if nargin && ischar(varargin{1})
    gui_State.gui_Callback = str2func(varargin{1});
end

if nargout
    [varargout{1:nargout}] = gui_mainfcn(gui_State, varargin{:});
else
    gui_mainfcn(gui_State, varargin{:});
end
% End initialization code - DO NOT EDIT


% --- Executes just before GUI is made visible.
function GUI_OpeningFcn(hObject, eventdata, handles, varargin)
% This function has no output args, see OutputFcn.
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% varargin   command line arguments to GUI (see VARARGIN)

% Choose default command line output for GUI
handles.output = hObject;

% Update handles structure
guidata(hObject, handles);

% UIWAIT makes GUI wait for user response (see UIRESUME)
% uiwait(handles.figure1);


% --- Outputs from this function are returned to the command line.
function varargout = GUI_OutputFcn(hObject, eventdata, handles) 
% varargout  cell array for returning output args (see VARARGOUT);
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Get default command line output from handles structure
varargout{1} = handles.output;


% --- Executes on button press in openImage.
function openImage_Callback(hObject, eventdata, handles)
% hObject    handle to openImage (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
imageName = uigetfile();
handles.Image = imread(imageName);
axes(handles.axes1);


I = rgb2gray(handles.Image);

I = imgaussfilt(I,2);
rotI = I;

BW = edge(rotI,'canny');
se90 = strel('line', 3, 90);
se0 = strel('line', 3, 0);
BW = imdilate(BW,[se90 se0]);
figure,imshow(BW);
[H,T,R] = hough(BW, 'rho', 0.5,'Theta', -90:0.1:89.9);
%[H,T,R] = hough(BW);
figure,imshow(H,[],'XData',T,'YData',R,...
            'InitialMagnification','fit');
xlabel('\theta'), ylabel('\rho');
axis on, axis normal, hold on;

P  = houghpeaks(H,10,'threshold',ceil(0.3*max(H(:))));
x = T(P(:,2)); y = R(P(:,1));
plot(x,y,'s','color','white');
[s1, s2] = size(I);
s1 = sqrt(s1*s2);
lines = houghlines(BW,T,R,P,'FillGap',s1*0.1,'MinLength', s1*0.1);
figure, imshow(rotI), hold on
max_len = 0;
for k = 1:length(lines)
    lines(k).distance = norm(lines(k).point1 - lines(k).point2);
    
   xy = [lines(k).point1; lines(k).point2];
   plot(xy(:,1),xy(:,2),'LineWidth',2,'Color','green');

   % Plot beginnings and ends of lines
   plot(xy(1,1),xy(1,2),'x','LineWidth',2,'Color','yellow');
   plot(xy(2,1),xy(2,2),'x','LineWidth',2,'Color','red');

   % Determine the endpoints of the longest line segment
   len = norm(lines(k).point1 - lines(k).point2);
   if ( len > max_len)
      max_len = len;
      xy_long = xy;
   end
end
[values,index] = sort([lines.distance],'descend');
values
index

for i=1:length(lines)
    for j=i:length(lines)
        first = lines(index(i));
        second = lines(index(j));
        x=min([abs(first.point1-second.point1);abs(first.point2-second.point1);abs(first.point1-second.point2); ... 
            abs(first.point2-second.point2)]);
        if (abs(first.theta-second.theta)<=100 && abs(first.theta-second.theta)>=80 && abs(first.distance-second.distance)<=30 ...
            && x(1)<=50 && x(2)<=50)
            minDis = min(first.distance, second.distance);
            
            figure, imshow(imcrop(handles.Image, [min([second.point1(1);second.point2(1);first.point1(1); first.point2(1)]) min([second.point1(2);second.point2(2);first.point1(2); first.point2(2)]) minDis minDis])), hold on;
            xy=[first.point1 ; first.point2];
            %plot(xy(:,1),xy(:,2),'LineWidth',2,'Color','green');
            xy=[second.point1 ; second.point2];
            %plot(xy(:,1),xy(:,2),'LineWidth',2,'Color','red');
            first
            second
        end
    end
end

%plot(xy_long(:,1),xy_long(:,2),'LineWidth',2,'Color','cyan');


%{
I = imgaussfilt(I,2);
imshow(I);

[~, threshold] = edge(I, 'sobel');
fudgeFactor = .6;
BWs = edge(I,'sobel', threshold * fudgeFactor);
figure,imshow(BWs);

se90 = strel('line', 5, 90);
se0 = strel('line', 5, 0);

BWsdil = imdilate(BWs, [se90 se0]);
figure,imshow(BWsdil);

BWsdil = imerode(BWsdil, [se90 se0]);
figure,imshow(BWsdil);

se90 = strel('line', 3, 90);
se0 = strel('line', 3, 0);
seSQ = strel('square', 5);
BWsdil = imdilate(BWs, seSQ);
figure,imshow(BWsdil);

BWdfill = imfill(BWsdil, 'holes');
figure, imshow(BWdfill);

seD = strel('square',2);
BWfinal = imerode(BWdfill,seD);
BWfinal = imerode(BWfinal,seD);
figure, imshow(BWfinal);

BWoutline = bwperim(BWfinal);
Segout = I;
Segout(BWoutline) = 0;
figure, imshow(Segout);
%}