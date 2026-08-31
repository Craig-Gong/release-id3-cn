import os

from iqpilot.common.basedir import BASEDIR
from iqdbc.car.docs import generate_cars_md, get_all_car_docs
from iqdbc.lvbs.car.car_catalog import build_car_catalog
from iqpilot.selfdrive.debug.dump_car_docs import dump_car_docs
from iqpilot.selfdrive.debug.print_docs_diff import print_car_docs_diff
from iqpilot.selfdrive.car.docs import CARS_MD_TEMPLATE
from iqpilot.selfdrive.car.vehicle_catalog import load_catalog


class TestCarDocs:
  @classmethod
  def setup_class(cls):
    cls.all_cars = get_all_car_docs()

  def test_generator(self):
    generate_cars_md(self.all_cars, CARS_MD_TEMPLATE)

  def test_docs_diff(self):
    dump_path = os.path.join(BASEDIR, "iqpilot", "selfdrive", "car", "tests", "cars_dump")
    dump_car_docs(dump_path)
    print_car_docs_diff(dump_path)
    os.remove(dump_path)

  def test_vehicle_catalog(self):
    assert load_catalog() == build_car_catalog()
