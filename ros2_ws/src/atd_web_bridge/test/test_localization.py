from types import SimpleNamespace as NS

from atd_web_bridge.core import Cache
from atd_web_bridge.localization import live_observations, observation


def sample(state='tracking'):
    return NS(state=state, map_id='test', map_sha256='a'*64, hdmap_sha256='b'*64, session_id='launch',
              epoch=3, reason='test', header=NS(stamp=NS(sec=10, nanosec=0), frame_id='map'),
              pose=NS(position=NS(x=2., y=-1., z=0.), orientation=NS(w=1., x=0., y=0., z=0.)),
              speed_mps=-.2, yaw_rate_rps=.3)


def test_live_uses_single_coherent_source_and_expires_it():
    cache = Cache(2.)
    cache.put('pose', dict(x=999))
    assert live_observations(cache, 0, 10.)['pose']['x'] == 999
    cache.put('localization', observation(sample()))
    result = live_observations(cache, 1, 10.5)
    assert result['pose']['x'] == 2. and result['velocity']['linear_mps'] == -.2
    assert result['localization']['remaining_s'] == 1.5
    for n, clock in [(2, 10.), (0, 10.), (1, 12.5), (1, 2.)]:
        stale = live_observations(cache, n, clock)
        assert stale['localization']['state'] == 'stale' and stale['pose'] is None and stale['velocity'] is None
    cache.put('localization', observation(sample('lost')))
    assert live_observations(cache, 1, 10.)['pose'] is None


def test_invalid_pose_or_identity_has_no_readings():
    msg = sample()
    msg.pose.orientation.w = 0
    assert observation(msg)['state'] == 'stale'
    msg = sample()
    msg.hdmap_sha256 = ''
    assert observation(msg)['state'] == 'stale'
