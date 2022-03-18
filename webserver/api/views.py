import json
from django.shortcuts import render
from django.http import HttpResponse
from rest_framework import generics, status

from .serializers import UserSerializer, LiveDataSerializer, InitLiveDataSerializer
from .models import User, LiveData

from rest_framework.views import APIView
from rest_framework.response import Response

from ast import literal_eval

# Create your views here.
class UserView(generics.ListAPIView):
    queryset = User.objects.all()
    serializer_class = UserSerializer

"""
CreateLiveSession
Given a username it creates a live data session with a unique code and initializes
all sensor values to 0. Used by the GUI user
"""
class CreateLiveSession(APIView):
    serializer_class = InitLiveDataSerializer

    def post(self, request, format=None):
        if not self.request.session.exists(self.request.session.session_key):
            self.request.session.create()
        print(request.data)
        serializer = self.serializer_class(data=request.data)
        if serializer.is_valid():
            USERNAME = serializer.data.get('username')
            ECG = [0]
            EMG = [0]
            FORCE = [0]
            GYRO_X = [0]
            GYRO_Y = [0]
            GYRO_Z = [0]
            ACCEL_X = [0]
            ACCEL_Y = [0]
            ACCEL_Z = [0]
            live_data = LiveData(username=USERNAME, ecg=ECG, emg=EMG, force=FORCE, gyro_x=GYRO_X, gyro_y=GYRO_Y, gyro_z=GYRO_Z, accel_x=ACCEL_X, accel_y=ACCEL_Y, accel_z=ACCEL_Z)
            live_data.save()
            self.request.session['uniqueCode'] = live_data.uniqueCode
            return Response(LiveDataSerializer(live_data).data, status=status.HTTP_201_CREATED)
        return Response({'Bad Request': 'Invalid data...'}, status=status.HTTP_400_BAD_REQUEST)

# class GetECG(APIView):
#     serializer_class = LiveDataSerializer
#     lookup_url_kwarg = 'id'

#     def get(self, request, format=None):
#         id = request.GET.get(self.lookup_url_kwarg)
#         if id != None:
#             live_data = LiveData.objects.filter(uniqueCode=id)
#             if len(live_data) > 0:
#                 data = LiveDataSerializer(live_data[0]).data

"""
GetUserCode 
args: 
    username: the username of the user whose data you want to send
returns a user's unique session code given their username.
Used by the MCU to get the code to send
"""
class GetUserCode(APIView):
    serializer_class = LiveDataSerializer
    lookup_url_kwarg = 'username'

    def get(self, request, format=None):
        username = request.GET.get(self.lookup_url_kwarg)
        if username != None:
            user = LiveData.objects.filter(username=username)
            if len(user) > 0:
                data = LiveDataSerializer(user[0]).data
                return Response({'uniqueCode': data['uniqueCode']}, status=status.HTTP_200_OK)
            return Response({'User not found': 'invalid username'}, status=status.HTTP_404_NOT_FOUND)
        return Response({'Bad request': 'username parameter not found in request'}, status=status.HTTP_400_BAD_REQUEST)

"""
SendData
Used by the MCU as a post request to add new sensor data to be displayed.
"""
class SendData(APIView):

    def post(self, request, format=None):
        querySet = LiveData.objects.filter(uniqueCode=request.data['uniqueCode'])
        if querySet.exists():
            user = querySet[0]
            userdata = LiveDataSerializer(user).data #existing data to add to

            udata_ecg = literal_eval(userdata['ecg'])
            for ecg in request.data.getlist('ECG'):
                udata_ecg.append(literal_eval(ecg))
            
            udata_emg = literal_eval(userdata['emg'])
            for emg in request.data.getlist('EMG'):
                udata_emg.append(literal_eval(emg))
            
            udata_force = literal_eval(userdata['force'])
            udata_force.append(int(float(request.data['FORCE'])))
            
            udata_gyro_x = literal_eval(userdata['gyro_x'])
            udata_gyro_x.append(int(float(request.data['GYRO_X'])))

            udata_gyro_y = literal_eval(userdata['gyro_y'])
            udata_gyro_y.append(int(float(request.data['GYRO_Y'])))

            udata_gyro_z = literal_eval(userdata['gyro_z'])
            udata_gyro_z.append(int(float(request.data['GYRO_Z'])))

            udata_accel_x = literal_eval(userdata['accel_x'])
            udata_accel_x.append(int(float(request.data['ACCEL_X'])))

            udata_accel_y = literal_eval(userdata['accel_y'])
            udata_accel_y.append(int(float(request.data['ACCEL_Y'])))

            udata_accel_z = literal_eval(userdata['accel_z'])
            udata_accel_z.append(int(float(request.data['ACCEL_Z'])))
            
            user.ecg = udata_ecg
            user.emg = udata_emg
            user.force = udata_force
            user.gyro_x = udata_gyro_x
            user.gyro_y = udata_gyro_y
            user.gyro_z = udata_gyro_z
            user.accel_x = udata_accel_x
            user.accel_y = udata_accel_y
            user.accel_z = udata_accel_z
            user.save()
            return Response(LiveDataSerializer(user).data, status=status.HTTP_202_ACCEPTED)
        else:
            return Response({'Bad Request': 'Invalid unique code...'}, status=status.HTTP_400_BAD_REQUEST)
       

"""
GetData
Used by graph function to fetch data from a single field to display
"""
class GetData(APIView):
    lookup_url_kwarg = 'data'
    lookup_url_kwarg2 = 'uniqueCode'

    def get(self, request, format=None):
        data_to_find = request.GET.get(self.lookup_url_kwarg)
        id = request.GET.get(self.lookup_url_kwarg2)
        if id != None:
            user = LiveData.objects.filter(uniqueCode=id)
            if len(user) > 0:
                user_data = LiveDataSerializer(user[0]).data
                udata = literal_eval(user_data[data_to_find])
                data_point = udata[0]
                if len(udata) > 1:
                    udata.pop(0)
                user[0].ecg = udata
                user[0].save()
                return Response({'data': data_point}, status=status.HTTP_200_OK)
            return Response({'User not found': 'invalid uniqueCode'}, status=status.HTTP_404_NOT_FOUND)
        return Response({'Bad request': 'uniqueCode parameter not found in request'}, status=status.HTTP_400_BAD_REQUEST)
                




"""
simple get endpoint to test the api by returning a simple dictionary
"""
class TestAPI(APIView):
    serializer_class = LiveDataSerializer

    def get(self, request, format=None):
        return Response({'one': 1, 'two': 2, 'three': 4}, status=status.HTTP_202_ACCEPTED)
