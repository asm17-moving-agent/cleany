import numpy as np
import pytest

from cleany_grasping.core.models import PointCloud
from cleany_grasping.core.top_face import TopFace
from cleany_grasping.geometric_predictor import GeometricGraspConfig, GeometricGraspPredictor

UP = np.array([0., 0., 1.])
ROT = np.column_stack((-UP, [0., 1., 0.], [1., 0., 0.]))


def cloud(points):
    values = np.asarray(points, dtype=float)
    return PointCloud(values, np.zeros_like(values))


def grid_face(width_at_x, z=.040):
    return np.asarray([(x, y, z) for x in np.linspace(-.050, .050, 41)
                       for y in np.linspace(-width_at_x(x)/2, width_at_x(x)/2, 31)])


def measure(points, site=0., half_span=.012):
    body = cloud(points)
    face = TopFace.fit(body, body, UP, .0015)
    assert face is not None
    return face.contact(body.points, np.array([site, 0., .03]), ROT, half_span,
                        .009, .003, .022, .008, .5)


@pytest.mark.parametrize('width', [.030, .050, .078])
def test_flat_top_ignores_the_wider_base_below_the_fingers(width):
    roof = grid_face(lambda x: width)
    base = grid_face(lambda x: width+.016, z=.008)[::4]
    contact = measure(np.vstack([roof, base]))
    assert contact.face_width_m == pytest.approx(width, abs=.001)
    assert contact.opening_m == pytest.approx(width+.008, abs=.001)
    assert contact.center[2] == pytest.approx(.046)


def test_wider_shoulder_inside_fingertip_sweep_keeps_its_clearance():
    roof = grid_face(lambda x: .044)
    lip = grid_face(lambda x: .070, z=.035)[::5]
    contact = measure(np.vstack((roof, lip)))
    assert contact.face_width_m == pytest.approx(.044, abs=.001)
    assert contact.opening_m >= .070 + .008 - .0001


def test_taper_measures_the_full_pad_strip_at_each_site():
    points = grid_face(lambda x: .050+.50*x)
    for site in (-.025, .025):
        contact = measure(points, site, half_span=.010)
        # The wider end of each strip, not the local centerline, sets the width.
        assert contact.face_width_m == pytest.approx(.050+.50*(site+.010), abs=.002)


@pytest.mark.parametrize('size', [.012, .060])
def test_round_side_keeps_body_width_in_the_normal_predictor(size):
    angles = np.linspace(0., np.pi, 73)
    points = np.array([(x, size/2*np.cos(t), size/2*(1+np.sin(t)))
                       for x in np.linspace(-.05, .05, 41) for t in angles])
    table = cloud([[x, y, 0.] for x in np.linspace(-.15,.15,31) for y in np.linspace(-.15,.15,31)])
    config = GeometricGraspConfig(top_face_tolerance_m=.0015, top_face_tip_reach_m=.022,
                                  pad_half_length_m=.015375, distal_pad_offset_m=.009, yaw_offsets_degrees=(0.,))
    body = cloud(points)
    assert TopFace.fit(body, table, UP, .0015) is None
    grasps = GeometricGraspPredictor(config).predict(body, table, np.zeros(6))
    assert grasps and grasps[0].width_m-config.opening_margin_m == pytest.approx(size,abs=.001)


def test_disk_has_a_diameter_not_just_the_visible_sidewall_chord():
    r=.029
    xy=np.array([(x,y) for x in np.linspace(-r,r,55) for y in np.linspace(-r,r,55)
                 if x*x+y*y <= r*r+1e-12])
    disk=np.column_stack((xy,np.full(len(xy),.04)))
    contact=measure(disk)
    assert contact.face_width_m == pytest.approx(2*r,abs=.003)


def test_sparse_or_disconnected_rim_does_not_bridge_an_empty_contact():
    thin = grid_face(lambda x: .05)
    body = cloud(thin[:7])
    assert TopFace.fit(body,body,UP,.0015) is None
    rails=thin[abs(thin[:,1])>.019]
    assert measure(rails) is None


def test_noisy_edge_depth_and_a_visible_shoulder_do_not_define_the_flat_top():
    rng = np.random.default_rng(17)
    roof = grid_face(lambda x: .056)
    roof[:, 2] += rng.normal(0., .00035, len(roof))
    left = np.array([(x, -.047, .012) for x in np.linspace(-.046,.046,13)])
    right = grid_face(lambda x: .078, z=.025)[::4]
    contact = measure(np.vstack((roof, left, right)))
    assert contact.face_width_m == pytest.approx(.056, abs=.001)
    assert contact.opening_m >= .078 + .008 - .0001


def test_curved_sphere_does_not_create_a_narrow_top_face():
    radius=.034
    az, el=np.meshgrid(np.linspace(0, 2*np.pi, 90, endpoint=False),
                       np.linspace(0, np.pi/2, 35, endpoint=False))
    sphere=cloud(radius*np.column_stack((np.cos(el).ravel()*np.cos(az).ravel(),
                                         np.cos(el).ravel()*np.sin(az).ravel(),
                                         1+np.sin(el).ravel())))
    assert TopFace.fit(sphere, sphere, UP, .0015) is None


def test_no_colour_contrast_cannot_extend_a_top_surface():
    roof = grid_face(lambda x: .05)
    body = cloud(roof)
    table = grid_face(lambda x: .18,z=.001)
    context = PointCloud(table, np.zeros_like(table), viewpoint=np.array([0.,-.35,.5]))
    face = TopFace.fit(body, context, UP, .0015)
    assert face is not None
    assert np.array_equal(face.points,body.points)
