#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
import launch.actions
import launch_ros.actions


# Please note that this is only an example!
# It is not guaranteed to work with your setup but can be used as a starting point.


def generate_launch_description():

    parameters_file = os.path.join(
        get_package_share_directory('racecar'),
        'config', 'joy_teleop.yaml'
    )

    ld = LaunchDescription([
        launch.actions.DeclareLaunchArgument('cmd_vel', default_value='input_joy/cmd_vel'),
        launch.actions.DeclareLaunchArgument('teleop_config', default_value=parameters_file),
    ])

    ld.add_action(launch_ros.actions.Node(
            package='racecar', executable='joy_teleop',
            parameters=[launch.substitutions.LaunchConfiguration('teleop_config')]))

    ld.add_action(launch_ros.actions.Node(
            package='racecar', executable='incrementer_server',
            name='incrementer', namespace='torso_controller'))

    return ld