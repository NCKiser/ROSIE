from django.urls import path

from . import views

app_name = 'control'
urlpatterns = [
    path('', views.index, name='index'),
    path('<int:quality>/', views.indexQ, name='indexQ'),
    path('stream/', views.stream, name='stream'),
    path('stream/<int:quality>/', views.custom, name='custom'),
    path('command/', views.command, name='command'),
    path('stoprecord/', views.stoprecord, name='stoprecord'),
    path('startrecord/', views.startrecord, name='startrecord'),
]
