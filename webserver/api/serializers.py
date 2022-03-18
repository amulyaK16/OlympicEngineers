from rest_framework import serializers
from .models import User
from .models import LiveData

class UserSerializer(serializers.ModelSerializer):
    class Meta:
        model = User
        fields = ('id', 'username', 'password', 'email', 'created_at')

class LiveDataSerializer(serializers.ModelSerializer):
    class Meta:
        model = LiveData
        fields = ('username', 'uniqueCode', 'ecg', 'emg', 'force', 'gyro_x', 'gyro_y', 'gyro_z', 'accel_x', 'accel_y', 'accel_z')

class InitLiveDataSerializer(serializers.ModelSerializer):
    class Meta:
        model = LiveData
        fields = ('username',)