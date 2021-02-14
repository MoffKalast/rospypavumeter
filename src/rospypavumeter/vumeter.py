#!/usr/bin/python
import sys
import argparse
import time
from Queue import Queue
from ctypes import POINTER, c_ubyte, c_void_p, c_ulong, cast
from datetime import datetime, timedelta
import time

# From https://github.com/Valodim/python-pulseaudio
from pulseaudio.lib_pulseaudio import *

# ROS
import rospy
from std_msgs.msg import UInt8

# ros node name
_DEFAULT_NAME = 'vumeter_node'

class PeakMonitor(object):

	def __init__(self, rate):
		self.rate = rate
		# This boolean controls the correct setup of the vumeter
		self.pa_state = False
		self.timestamp = datetime.now()
		self.timestamp_stream = datetime.now()

		# Wrap callback methods in appropriate ctypefunc instances so
		# that the Pulseaudio C API can call them
		self._context_notify_cb = pa_context_notify_cb_t(self.context_notify_cb)
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
		# Creates the context, which multiplexes commands, data streams and events through a single channel
		# It also sets the callback detecting any modifications in its state
		context = pa_context_new(_mainloop_api, 'peak_demo')
		pa_context_set_state_callback(context, self._context_notify_cb, None)

		# Sets the samplespec and creates the stream
		self.samplespec = pa_sample_spec()
		self.samplespec.channels = 1
		self.samplespec.format = PA_SAMPLE_U8
		self.samplespec.rate = self.rate
		# Stream inicialization
		self.pa_stream = pa_stream_new(context, "peak detect demo", self.samplespec, None)
		
		pa_context_connect(context, None, 0, None)
		pa_threaded_mainloop_start(_mainloop)

	##
	## @brief      Iterator function. An iterator in Python is simply an object that can be iterated upon.
	##             An object which will return data, one element at a time. This function stores all the values 
	##             stored in the sample queue in an iterable object, a generator
	##
	## @param      self  The object
	##
	## @return     This function stores all the values 
	##             stored in the sample queue in an iterable object, a generator, which is returned
	##
	def __iter__(self):
		while True:
			yield self._samples.get()

	##
	## @brief      This callback triggers when it is perceive any change in the context state,
	##             in this case, upon inicialization
	##
	## @param      self     The object
	## @param      context  A context is the basic object for a connection to a PulseAudio server. 
	##                      It multiplexes commands, data streams and events through a single channel.
	## @param      _        This is a placeholder for userdata
	##
	## @return     Does not return any value
	##
	def context_notify_cb(self, context, _):
		
		state = pa_context_get_state(context)

		# When the context is ready
		if state == PA_CONTEXT_READY:
			rospy.loginfo("Pulseaudio connection ready...")
			# Connected to Pulseaudio. Now request that sink_info_cb
			# be called with information about the available sinks.

			# Get the sinks info and tries to connect to one of them once the context is ready
			o = pa_context_get_sink_info_list(context, self._sink_info_cb, None)
			pa_operation_unref(o)

			# Create the subscription calllbacks in order to catch events related to modifications in 
			# soundcards, sinks and sources. This events are stored in a mask. In this particular case
			# the events selected are the ones related with the sink and the soundcards 
			mask = PA_SUBSCRIPTION_MASK_SINK | PA_SUBSCRIPTION_MASK_SINK_INPUT | PA_SUBSCRIPTION_MASK_CARD
			pa_context_set_subscribe_callback(context, self._context_subscribe_cb, None)
			o = pa_context_subscribe(context, mask, self._context_success_cb, None)
			pa_operation_unref(o)

		# In the case the context is unable to connect. Notifies through terminal.
		elif state == PA_CONTEXT_FAILED :
			rospy.loginfo("Connection failed")

		# If the process has finished, it just notifies via terminal.
		elif state == PA_CONTEXT_TERMINATED:
			rospy.loginfo("Connection terminated")


	##
	## @brief      The callback function that is called when new data is
	##             available from the stream.
	##
	## @param      self    The object
	## @param      stream  An opaque stream for playback or recording.
	## @param      length  The length of the received data
	## @param      _       Placeholder for userdata
	##
	## @return     Does not return anything
	##
	def stream_read_cb(self, stream, length, _):
		#if datetime.now() - self.timestamp_stream > timedelta(days=0,seconds=0.2):
		data = c_void_p()
		pa_stream_peek(stream, data, c_ulong(length))
		data = cast(data, POINTER(c_ubyte))
		for i in xrange(length):
			# When PA_SAMPLE_U8 is used, samples values range from 128
			# to 255 because the underlying audio data is signed but
			# it doesn't make sense to return signed peaks.
			self._samples.put(data[i] - 128)
		self.timestamp_stream = datetime.now()
		pa_stream_drop(stream)


	##
	## @brief      Context specific callback function that is called whenever the state of the daemon changes.
	##
	## @param      self       The object
	## @param      context    The context
	## @param      event_typ  An integer indicating the event type
	## @param      idex       The id of the operation
	## @param      _          Placeholder for userdata
	##
	## @return     Does not return any value
	##
	def context_subscribe_cb(self, context, event_typ, idex, _):
		
		event_type_masked = event_typ & PA_SUBSCRIPTION_EVENT_TYPE_MASK

		# If the event is related with a sink input event. This event, specifically in conjuction with a
		# sink change event, allows the system to initialize, and properly set the active sink.
		# The vumeter will wait until this sequence of events happens, meaning that there is active sink 
		# reproducing audio, and will choose this sink as the active sink.
		if (event_typ & PA_SUBSCRIPTION_EVENT_FACILITY_MASK) == PA_SUBSCRIPTION_EVENT_SINK_INPUT:
					
			if event_type_masked == PA_SUBSCRIPTION_EVENT_NEW and self.pa_state == False:
				# Connect this event with the Sink Change event, if they happen in sequence, the system becomes
				# active from its initial suspended state and chooses the active sink.
				self.timestamp = datetime.now()
				
		# This statement catches events related with the sinks
		# In this case the event catched is meant to occcur after the Sink Input event, in the system initial state,
		# waking the system up and choosing an active sink. 
		elif (event_typ & PA_SUBSCRIPTION_EVENT_FACILITY_MASK) == PA_SUBSCRIPTION_EVENT_SINK:
				   
			if event_type_masked == PA_SUBSCRIPTION_EVENT_CHANGE and self.pa_state == False:
				#print "Changes detected in the Sink. Comparing the timestamp..."
				if datetime.now() - self.timestamp < timedelta(days=0,seconds=2):
					#print "Enough time has passed since the timestamp was recorded"
					pa_stream_disconnect(self.pa_stream)
					o = pa_context_get_sink_info_list(context, self._sink_info_cb, None)
					pa_operation_unref(o)

		# This statement catches any modifications made to the soundcards as additions, removals, etc
		# Card-related events cannot occur at the same time, what makes the system easier to be controlled
		elif (event_typ & PA_SUBSCRIPTION_EVENT_FACILITY_MASK) == PA_SUBSCRIPTION_EVENT_CARD:
			#print "Card event detected"
			pa_stream_disconnect(self.pa_stream)
			time.sleep(1)
			o = pa_context_get_sink_info_list(context, self._sink_info_cb, None)
			pa_operation_unref(o)
   
	##
	## @brief      A generic callback for operation completion. 
	##             Has to be defined in order to the context to be able to subscribe  
	##
	## @param      self       The object
	## @param      context    The context
	## @param      succedded  Integer indicating the success of the operation
	## @param      _          Placeholder for userdata
	##
	## @return     Does not returns
	##
	def context_success_cb(self, context, succedded, _):
		rospy.loginfo('Subscribe operation completed')

	##
	## @brief      Get some information about a sink. If the sink is running, sets it as the active sink, 
	##             connecting it to a stream 
	##
	## @param      self         The object
	## @param      context      The context
	## @param      sink_info_p  Structure that stores info about sinks
	## @param      _            Placeholder for eol integer value
	## @param      __           Placeholder for userdata
	##
	## @return     As a callback, does not returns
	##
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

		# Checks if the sink is currently running
		if sink_info.state == 0:
			# Found the sink we want to monitor for peak levels.
			# Tell PA to call stream_read_cb with peak samples.
			rospy.loginfo('Monitoring sink with name \033[1m%s\033[0m', sink_info.name)
			pa_stream_disconnect(self.pa_stream)
			self.pa_stream = pa_stream_new(context, "peak detect demo", self.samplespec, None)
			self.pa_state = True

			# Setting the stream reading and recording callbacks
			pa_stream_set_read_callback(self.pa_stream,
										self._stream_read_cb,
										sink_info.index)
			pa_stream_connect_record(self.pa_stream,
									 sink_info.monitor_source_name,
									 None,
									 PA_STREAM_PEAK_DETECT)


def _init_node(node_name):
	"""
	Common routines for a start node inside ROS environment
	
	@param      node_name  The node name
	
	@return     Does not return any value
	"""
	rospy.init_node(node_name)
	rospy.loginfo("Initializing {} Node".format(rospy.get_name()))

##
## @brief      This function publishes the peaks detected by the vumeter through ROS communication protocol
##
## @param      _METER_RATE        The meter rate
## @param      _MAX_SAMPLE_VALUE  The max sample value
## @param      _DISPLAY_SCALE     The display scale
##
## @return     
##
def _audio_level_publisher(_METER_RATE, _MAX_SAMPLE_VALUE, _DISPLAY_SCALE):

	audioLevelPublisher = rospy.Publisher('audio_level', UInt8, queue_size=70)
	rate = rospy.Rate(_METER_RATE)
	
	monitor = PeakMonitor(_METER_RATE)
	
	prevlevel = 0;
	while not rospy.is_shutdown():
		
		level = monitor._samples.get() >> _DISPLAY_SCALE
		level = level * 0.6
		#print level
		#bar = '|' * level
		#spaces = ' ' * ((_MAX_SAMPLE_VALUE >> _DISPLAY_SCALE) - level)	
		#explicit_level = ' %3d %s%s\r' % (level, bar, spaces)
		
		#rospy.loginfo(explicit_level)
		#rospy.loginfo('Level published =  %d', level)
		if int(prevlevel) != 0:
			audioLevelPublisher.publish(level)

		prevlevel = level


		#rate.sleep()

if __name__ == '__main__':
	
	_init_node(_DEFAULT_NAME)

	METER_RATE = rospy.get_param("meter_rate")
	MAX_SAMPLE_VALUE = rospy.get_param("max_sample_value")
	DISPLAY_SCALE = rospy.get_param("display_scale")

	try:
		_audio_level_publisher(METER_RATE, MAX_SAMPLE_VALUE, DISPLAY_SCALE)
		
	except rospy.ROSInterruptException:
		pass
