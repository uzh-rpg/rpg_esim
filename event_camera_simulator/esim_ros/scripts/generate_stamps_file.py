import argparse
from os import listdir
from os.path import join


if __name__ == "__main__":

    parser = argparse.ArgumentParser(
        description='Generate "images.csv" for ESIM DataProviderFromFolder')

    parser.add_argument('-i', '--input_folder', default=None, type=str,
                        help="folder containing the images")
    parser.add_argument('-r', '--framerate', default=1000, type=float,
                        help="video framerate, in Hz")

    args = parser.parse_args()

    images = sorted(
        [f for f in listdir(args.input_folder) if f.endswith('.png')])

    print('Will write file: {} with framerate: {} Hz'.format(
        join(args.input_folder, 'images.csv'), args.framerate))
    stamp_nanoseconds = 1
    dt_nanoseconds = int((1.0 / args.framerate) * 1e9)
    with open(join(args.input_folder, 'images.csv'), 'w') as f:
        for image_path in images:
            f.write('{},{}\n'.format(stamp_nanoseconds, image_path))
            stamp_nanoseconds += dt_nanoseconds

    print('Done!')
