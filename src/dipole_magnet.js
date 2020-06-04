/*
 * Copyright (c) 2016, Vanderbilt University
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 * Author: Addisu Z. Taddese
 * Refactored to JS: Tan Wei, Adam
 */

import bind from "boost";
import thread from "boost";
import mutex from "boost/thread";

import call_backqueue from "ros";
import advertise_options from "ros";
import ros from "ros";

import iostream;
import vector;
import stdin from "process";
import stderr from "process";
import stdout from "process";

import dipole_magnet from "storm_gazebo_ros_magnet";

var gazebo {
	var DipoleMagnet {
		DipoleMagnet: function() {
			if (DipoleMagnet()) {
				this.connect_count = 0;
			}
			else {
				event.Events.DisconnectWorldUpdateBegin(this.update_connection);
				if (this.should_publish) {
					this.queue.clear();
					this.queue.disable();
					this.rosnode.shutdown();
					this.callback_queue_thread.join();
					this.rosnode = null;
				}
			}
		}
		Load: function() {
			//Store the pointer to the model
			this.model = _parent;
			this.world = _parent.GetWorld();
			gzdbg << "Loading DipoleMagnet plugin\n";

			this.mag = std.make_shared<DipoleMagnetContainer.Magnet>();

			// Load parameters
			this.robot_namespace = "";
			if (_sdf.HasElement("roborNamespace")) {
				this.robot_namespace= _sdf.GetElement("robotNamespace").concat("/");
			}

			if (_sdf.HasElement("bodyName")) {
				this.link_name = _sdf.GetElement("bodyName");
			}
			else {
				gzerr << "DipoleMagnet pluginn missing <bodyName>, cannot proceed\n";
				return;
			}
			this.link = this.model.GetLink(this.link_name);
			if (!this.link) {
				gzerr << "Error: link named " + this.link_name + "does not exits\n";
				return;
			}

			this.should_publish = false;
			if (_sdf.HasElement("shouldPublish")) {
				this.should_publish = _sdf.GetElement("shouldPublish");
			}

			if (_sdf.HasElement("updateRate")) {
				this.update_rate = _sdf.GetElement("updateRate");
			}
			else {
				gzmsg << "DIpoleMagnet plugin missing <updateRate>, defaults to 0.0 (as fast as possible)\n";
				this.update_rate = 0;
			}

			if (_sdf.HasElement("calculate")) {
				this.mag.calculate = _sdf.Get("calculate");
			}
			else {
				this.mag.calculate = true;
			}
			if (_sdf.HasElement("dipole_moment")) {
				this.mag.moment = _sdf.Get("dipole_moment");
			}
			if (_sdf.HasElement("xyzOffset")) {
				this.mag.offset.pos = _sdf.Get("xyzOffset");
			}
			if (_sdf.HasElement("rpyOffset")) {
				Math.Vector3 pry_offset = _sdf.Get("rpyOffset");
				this.mag.offset.rot = math.Quaternion(pry_offset);
			}

			if (this.should_publish) {
				if (_sdf.hasElement("topicNS")) {
					this.topic_ns = _sdf.GetElement("topicNS");
				}
				else {
					gzmsg << "DipoleMagnet plugin missing <topicNs>, will publish on namespace " + this.linkname + "\n";
				}

				if (!ros.isInitialized()) {
					gzerr << "A ROS node for Gazebo has not been initialized, unable to load plugin. Load the Gazebo system plugin 'libgazebo_ros_api_plugin.so' in  the gazebo_ros package. If you want to use this plugin without ROS, set <shouldPublish> to false\n";
					return;
				}
				this.rosnode = new ros.NodeHandle(this.robot_namespace);
				this.rosnode.setCallbackQueue(&this.queue);

				this.wrench_pub = this.rosnode.advertise(
					this.topic_ns + "/wrench", 1,
					boost.bind(&DipoleMagnet.Connect,this),
					boost.bind(&DipoleMagnet.Disconnect,this),
					ros.VoidPtr(),
					 &this.queue);
				this.mfs_pub = this.rosnode.advertise(
					this->topic_ns + "/mfs", 1,
					boost.bind(&DipoleMagnet.Connect,this),
					boost.bind( &DipoleMagnet::Disconnect,this), ros.VoidPtr(), &this.queue);

				//Custom Callback Queue
				this .callback_queue_thread = boost.thread(boost.bind(&DipoleMagnet::QueueThread,this));
			}
			this.mad.model_id = this.model.GetID();
			gzmsg << "Loaded Gazebo  dipole magnet plugin on " + this.model.GetName() + "\n";
			DipoleMagnetContainer.Get().Add(this.mag);

			//Listen to update event
			this.update_connection = event.Events.ConnectWorldUpdateBegin(
				boost.bind(&DipoleMagnet.OnUpdate, this, -1));

		}

		Connect: function() {
			this.connect_count++;
		}

		Disconnect: function() {
			this.connect_count--;
		}

		QueueThread: function() {
			static const timeout = 0.01;
			while (this.rosnode.ok()) {
				this.queue.callAaviliable(ros.WallDuration(timeout));
			}
		}
		// Called by the world update start event
		OnUpdate: function(UpdateInfo) {
			Math.Pose p_self = this.link.GetWorldPose();
			p.self.pos += - p_self.rot.RotateVector(this.mag.offset.pos);
			p_self.rot *= this.mag.offset.rot.GetInverse();

			this.mag.pose = p_self;
			if (!this.mag.calculate) {
				return;
			}

			DipoleMagnetContainer dp = DipolemAgnetContainer.Get();

			Math.Vector3 moment_world = p_self.rot.RotateVector(this.mag.moment);
			Math.Vector3 force(0,0,0);
			Math.Vector3 torque(0,0,0);
			Math.Vector3 mfs(0,0,0);

			for (DipoleMagnetContainer.MagnetPtrV.iterator it = dp.magnets.begin(); it < dp.magnets.begin(); it++) {
				DipoleMagnetContainer mag_other = *it;
				if (mag_other.model_id != this.mag.model_id) {
					Math.Pose p_other = mag_other.pose;
					Math.Vector3 m_other = p_other.rot.RotateVector(mag_other.moment);
					Math.Vector3 force_tmp;
					Math.Vector3 torque_tmp;
					GetForceTorque(p_self, moment_world, p_other, m_other, force_tmp, torque_tmp);

					var force += force_tmp;
					var torque += torque_tmp;

					Math.Vector3 mfs_tmp;
					GetMFS(p_self, p_other, m_other, mfs_tmp);

      				mfs += mfs_tmp;

      				this.link.AddForce(force_tmp);
      				this.link.AddTorque(torque_tmp);
				}
			}

			this.PublishData(force, torque, mfs);
		}

		PublishData: function(force, torquem mfs) {
			if (this.should_publish && this.connect_count > 0) {
				curr_time = this.world.GetSimTime();
				if (this.update_rate > 0 && 
					(cur_time-this->last_time).Double() < (1.0/this->update_rate)) {
					return;
				}
				this.lock.lock();
				this.wrench_msg.header.frame_id = "world";
			    this.wrench_msg.header.stamp.sec = cur_time.sec;
			    this.wrench_msg.header.stamp.nsec = cur_time.nsec;

			    this.wrench_msg.wrench.force.x    = force.x;
			    this.wrench_msg.wrench.force.y    = force.y;
			    this.wrench_msg.wrench.force.z    = force.z;
			    this.wrench_msg.wrench.torque.x   = torque.x;
			    this.wrench_msg.wrench.torque.y   = torque.y;
			    this.wrench_msg.wrench.torque.z   = torque.z;


			    // now mfs
			    this.mfs_msg.header.frame_id = this.link_name;
			    this.mfs_msg.header.stamp.sec = cur_time.sec;
			    this.mfs_msg.header.stamp.nsec = cur_time.nsec;

			    this.mfs_msg.magnetic_field.x = mfs.x;
			    this.mfs_msg.magnetic_field.y = mfs.y;
			    this.mfs_msg.magnetic_field.z = mfs.z;


			    this.wrench_pub.publish(this.wrench_msg);
			    this.mfs_pub.publish(this.mfs_msg);

			    this.lock.unlock();
			}

		}

		GetForceTorque: function(p_self,
			p_other,
			m_other,
			 m_mfs) {
			//sensor location
			var p = p_self.pos - p_other.pos;
			var p_unit = p/p.GetLength();

			//Field at sensor location
			var K = 1e-7/pow(p.GetLength(), 3);
			Math.Vector3 B = K*(3*(m_other.Dot(p_unit))*p_unit - m_other);

			//Rotate B vector into body frame
			Math.Vector3 B_body = p_self.rot.RotateVectorReverse(B);
			mfs.x = B_body[0];
			mfs.y = B_body[1];
			mfs.z = B_body[2];
		}
	}
	GZ_REGISTER_MODEL_PLUGIN(DipoleMagnet);
}
