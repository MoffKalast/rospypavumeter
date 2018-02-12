#!/usr/bin/python
import sys
import argparse
from Queue import Queue
from ctypes import POINTER, c_ubyte, c_void_p, c_ulong, cast

# From https://github.com/Valodim/python-pulseaudio
from pulseaudio.lib_pulseaudio import *

# ROS
import rospy
from std_msgs.msg import UInt8

# ros node name
_DEFAULT_NAME = 'vumeter_node'

# edit to match the sink
#~ _SINK_NAME = 'alsa_output.pci-0000_00_1b.0.analog-stereo'
#~ _METER_RATE = 200 #344
#~ _MAX_SAMPLE_VALUE = 127
#~ _DISPLAY_SCALE = 0
#~ _MAX_SPACES = _MAX_SAMPLE_VALUE >> _DISPLAY_SCALE


class PeakMonitor(object):

    def __init__(self, sink_name, rate):
        self.sink_name = sink_name
        self.rate = rate

        # Wrap callback methods in appropriate ctypefunc instances so
        # that the Pulseaudio C API can call them
        self._context_notify_cb = pa_context_notify_cb_t(self.context_notify_cb)
        #self._context_event_cb = pa_context_event_cb_t(self.context_event_cb)
        self._context_subscribe_cb = pa_context_subscribe_cb_t(self.context_subscribe_cb)
        self._context_success_cb =  pa_context_success_cb_t(self.context_success_cb)
        self._sink_info_cb = pa_sink_info_cb_t(self.sink_info_cb)
        self._stream_read_cb = pa_stream_request_cb_t(self.stream_read_cb)    

        # stream_read_cb() puts peak samples into this Queue instance
        self._samples = Queue()

        # Create the mainloop thread and set our context_notify_cb
        # method to be called when there's updates relating to the
        # connection to Pulseaudio
        _mainloop = pa_threaded_mainloop_new()
        _mainloop_api = pa_threaded_mainloop_get_api(_mainloop)
        context = pa_context_new(_mainloop_api, 'peak_demo')
        pa_context_set_state_callback(context, self._context_notify_cb, None)
        #pa_context_set_event_callback(context, self._context_event_cb, None)

        # Sets the samplespec and creates the stream
        self.samplespec = pa_sample_spec()
        self.samplespec.channels = 1
        self.samplespec.format = PA_SAMPLE_U8
        self.samplespec.rate = self.rate
        self.pa_stream = pa_stream_new(context, "peak detect demo", self.samplespec, None)
        
        pa_context_connect(context, None, 0, None)
        pa_threaded_mainloop_start(_mainloop)

    def __iter__(self):
        while True:
            yield self._samples.get()

    def context_notify_cb(self, context, _):
        state = pa_context_get_state(context)

        if state == PA_CONTEXT_READY:
            rospy.loginfo("Pulseaudio connection ready...")
            # Connected to Pulseaudio. Now request that sink_info_cb
            # be called with information about the available sinks.
            o = pa_context_get_sink_info_list(context, self._sink_info_cb, None)
            pa_operation_unref(o)
            pa_context_set_subscribe_callback(context, self._context_subscribe_cb, None)
            mask = PA_SUBSCRIPTION_MASK_SINK_INPUT
            o = pa_context_subscribe(context, mask, self._context_success_cb, None)
            pa_operation_unref(o)

        elif state == PA_CONTEXT_FAILED :
            rospy.loginfo("Connection failed")

        elif state == PA_CONTEXT_TERMINATED:
            rospy.loginfo("Connection terminated")

    def sink_info_cb(self, context, sink_info_p, _, __):
        if not sink_info_p:
            return

        sink_info = sink_info_p.contents
        rospy.loginfo('\n' + ('-'* 60))
        #print '-'* 60
        rospy.loginfo('index: %d', sink_info.index)
        rospy.loginfo('name: \033[1m%s\033[0m', sink_info.name)
        rospy.loginfo('description: %s', sink_info.description)
        rospy.loginfo('state: %s', sink_info.state)

        if sink_info.state == 0:
            # Found the sink we want to monitor for peak levels.
            # Tell PA to call stream_read_cb with peak samples.
            rospy.loginfo('Monitoring sink with name \033[1m%s\033[0m', sink_info.name)
            pa_stream_disconnect(self.pa_stream)
            self.pa_stream = pa_stream_new(context, "peak detect demo", self.samplespec, None)
            pa_stream_set_read_callback(self.pa_stream,
                                        self._stream_read_cb,
                                        sink_info.index)
            pa_stream_connect_record(self.pa_stream,
                                     sink_info.monitor_source_name,
                                     None,
                                     PA_STREAM_PEAK_DETECT)

    def stream_read_cb(self, stream, length, index_incr):
        data = c_void_p()
        pa_stream_peek(stream, data, c_ulong(length))
        data = cast(data, POINTER(c_ubyte))
        for i in xrange(length):
            # When PA_SAMPLE_U8 is used, samples values range from 128
            # to 255 because the underlying audio data is signed but
            # it doesn't make sense to return signed peaks.
            self._samples.put(data[i] - 128)
        pa_stream_drop(stream)

    def context_subscribe_cb(self, context, event_typ, idex, __):
        print "The event is ", event_typ
        print "The id is ", idex

        pa_stream_disconnect(self.pa_stream)
        o = pa_context_get_sink_info_list(context, self._sink_info_cb, None)
        pa_operation_unref(o)

    def context_success_cb(self, context, succedded, _):
        rospy.loginfo('Subscribe operation completed')

def _init_node(node_name):
	"""Common routines for a start node."""
	rospy.init_node(node_name)
	rospy.loginfo("Initializing {} Node".format(rospy.get_name()))

def _audio_level_publisher(_SINK_NAME, _METER_RATE, _MAX_SAMPLE_VALUE, _DISPLAY_SCALE):

    audioLevelPublisher = rospy.Publisher('audioLevel', UInt8, queue_size=70)
    rate = rospy.Rate(_METER_RATE)
	
    monitor = PeakMonitor(_SINK_NAME, _METER_RATE)
	
    while not rospy.is_shutdown():
		
        level = monitor._samples.get() >> _DISPLAY_SCALE
		
		#bar = '|' * level
		#spaces = ' ' * ((_MAX_SAMPLE_VALUE >> _DISPLAY_SCALE) - level)	
		#explicit_level = ' %3d %s%s\r' % (level, bar, spaces)
		
		#rospy.loginfo(explicit_level)
        rospy.loginfo('Level published =  %d', level)
        audioLevelPublisher.publish(level)
        #rate.sleep()

if __name__ == '__main__':
	
    _init_node(_DEFAULT_NAME)

    SINK_NAME = rospy.get_param("sink_name")
    METER_RATE = rospy.get_param("meter_rate")
    MAX_SAMPLE_VALUE = rospy.get_param("max_sample_value")
    DISPLAY_SCALE = rospy.get_param("display_scale")

    try:
		_audio_level_publisher(SINK_NAME, METER_RATE, MAX_SAMPLE_VALUE, DISPLAY_SCALE)
        
    except rospy.ROSInterruptException:
		pass
