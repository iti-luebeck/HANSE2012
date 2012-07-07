#include "ace.h"

ACE::ACE()
{
    values = NULL;
    distances = NULL;
}

ACE::~ACE()
{
    if (values) cvReleaseMat(&values);
    if (distances) cvReleaseMat(&distances);
}

inline void randUni(CvMat *X, double l, double h) {
    for (int i = 0; i < X->rows; i++) {
        for (int j = 0; j < X->cols; j++) {
            double r = (double)rand();
            cvmSet(X, i, j, l + (h - l) * (r + 1) / RAND_MAX);
        }
    }
}

inline void randNormal(CvMat *X, double sigma, double mu) {
    CvMat *U1 = cvCreateMat(X->rows, X->cols, CV_64F);
    CvMat *U2 = cvCreateMat(X->rows, X->cols, CV_64F);

    randUni(U1, 0, 1);
    randUni(U2, 0, 1);

    for (int i = 0; i < X->rows; i++) {
        for (int j = 0; j < X->cols; j++) {
            double r = sqrt(-2 * log(cvmGet(U1, i, j)) / log(2));
            if (cvIsInf(r)) {
                r = 0;
            }
            double phi = 2 * CV_PI * cvmGet(U2, i, j);
            r = sigma * r * cos(phi) + mu;
            cvmSet(X, i, j, r);
        }
    }

    cvReleaseMat(&U1);
    cvReleaseMat(&U2);
}

void ACE::init(int width, int height, int nSamples, int sampleMethod, double stdev, int step, int colorSpace, int channels, int saturation)
{
    this->nSamples = nSamples;
    this->sampleMethod = sampleMethod;
    this->stdev = stdev;
    this->step = step;
    this->colorSpace = colorSpace;
    this->channels = channels;
    this->saturation = saturation;

    if (values) cvReleaseMat(&values);
    if (distances) cvReleaseMat(&distances);

    int k = 0;
    double nx = floor(sqrt((double)nSamples));
    double ny = ceil(sqrt((double)nSamples));
    switch (sampleMethod) {
    case ACE_SAMPLE_GRID:
        nSamples = nx * ny;
        this->nSamples = nSamples;
        values = cvCreateMat(2, nSamples, CV_32FC1);
        distances = cvCreateMat(1, nSamples, CV_32FC1);
        for (double i = 0; i < nx; i++) {
            for (double j = 0; j < ny; j++) {
                cvmSet(values, 0, k, (double)height * (i - (nx / 2)) / nx);
                cvmSet(values, 1, k, (double)width * (j - (ny / 2)) / ny);
                if (cvmGet(values, 0, i) == 0 && cvmGet(values, 1, k) == 0) {
                    cvmSet(distances, 0, k, 1);
                } else {
                    cvmSet(distances, 0, k, fabs(cvmGet(values, 0, k)) + fabs(cvmGet(values, 1, k)));
                }
                k++;
            }
        }
        break;
    case ACE_SAMPLE_UNIFORM:
        values = cvCreateMat(2, nSamples, CV_32FC1);
        randUni(values, -height, height);
        distances = cvCreateMat(1, nSamples, CV_32FC1);
        for (int i = 0; i < nSamples; i++) {
            if (cvmGet(values, 0, i) == 0 && cvmGet(values, 1, i) == 0) {
                cvmSet(distances, 0, i, 1);
            } else {
                cvmSet(distances, 0, i, fabs(cvmGet(values, 0, i)) + fabs(cvmGet(values, 1, i)));
            }
        }
        break;
    case ACE_SAMPLE_GAUSS:
        nSamples = nx * ny;
        this->nSamples = nSamples;

        values = cvCreateMat(2, nSamples, CV_32FC1);
        for (double i = 0; i < nx; i++) {
            for (double j = 0; j < ny; j++) {
                double x = (i - (nx / 2)) / (nx / 2);
                double y = (j - (ny / 2)) / (ny / 2);
                double p = exp(-0.5 * (x*x + y*y)) / sqrt(2 * M_PI);
                cvmSet(values, 0, k, stdev * (double)height * x / p);
                cvmSet(values, 1, k, stdev * (double)height * y / p);
                k++;
            }
        }

//        randNormal(values, stdev * width, 0);
        distances = cvCreateMat(1, nSamples, CV_32FC1);
        for (int i = 0; i < nSamples; i++) {
//            cvmSet(distances, 0, i, fabs(cvmGet(values, 0, i)) + fabs(cvmGet(values, 1, i)));
            if (cvmGet(values, 0, i) == 0 && cvmGet(values, 1, i) == 0) {
                cvmSet(distances, 0, i, 1);
            } else {
                cvmSet(distances, 0, i, fabs(cvmGet(values, 0, i)) + fabs(cvmGet(values, 1, i)));
            }
        }
        break;
    }
}

void ACE::automaticColorEqualization(IplImage *in, IplImage *out)
{
//    assert(in->width == out->width && in->height == in->height && in->nChannels == out->nChannels);
    int width = out->width;
    int height = out->height;
    CvMat *Rfilt;
    CvMat *R;
    if (colorSpace == ACE_COLORSPACE_GRAY) {
        R = cvCreateMat(height, width, CV_32FC1);
        Rfilt = cvCreateMat((int)ceil((double)height/step), (int)ceil((double)width/step), CV_32FC1);
    } else {
        R = cvCreateMat(height, width, CV_32FC3);
        Rfilt = cvCreateMat((int)ceil((double)height/step), (int)ceil((double)width/step), CV_32FC3);
    }
    float *rptr = R->data.fl;
    float *rfiltptr = Rfilt->data.fl;
    uchar *outptr = (uchar *)out->imageData;
    int outstep = out->widthStep;

    switch (colorSpace) {
    case ACE_COLORSPACE_HSV:
        cvCvtColor(in, out, CV_RGB2HLS);
        break;
    case ACE_COLORSPACE_YCrCb:
        cvCvtColor(in, out, CV_RGB2YCrCb);
        break;
    case ACE_COLORSPACE_RGB:
        cvCopy(in, out);
        break;
    case ACE_COLORSPACE_GRAY:
        cvCvtColor(in, out, CV_RGB2GRAY);
        break;
    }

    double maxR = 0;
    double fac = 256.0;
    double addFac = 10.0;
    double addResp = 1.0;
    for (int i = 0; i < height; i += step) {
        if (colorSpace == ACE_COLORSPACE_GRAY) {
            for(int j = 0; j < width; j += step) {
                rfiltptr[j/step] = (response(out, values, distances, i, j, 0) + addResp) / (((double)outptr[j] + addFac) / fac);
            }
            rfiltptr += Rfilt->width;
            outptr += step * outstep;
        } else {
            for(int j = 0; j < 3 * width; j += 3 * step) {
                if ((channels & ACE_CHANNEL_1) > 0) {
                    rfiltptr[j/step + 0] = (response(out, values, distances, i, j / 3, 0) + addResp) / (((double)outptr[j + 0] + addFac) / fac);
                }
                if ((channels & ACE_CHANNEL_2) > 0) {
                    rfiltptr[j/step + 1] = (response(out, values, distances, i, j / 3, 1) + addResp) / (((double)outptr[j + 1] + addFac) / fac);
                }
                if ((channels & ACE_CHANNEL_3) > 0) {
                    rfiltptr[j/step + 2] = (response(out, values, distances, i, j / 3, 2) + addResp) / (((double)outptr[j + 2] + addFac) / fac);
                }
            }
            rfiltptr += 3 * Rfilt->width;
            outptr += step * outstep;
        }
    }

    CvMat *Rfilt_1 = cvCreateMat(Rfilt->rows, Rfilt->cols, CV_32FC1);
    CvMat *Rfilt_2 = cvCreateMat(Rfilt->rows, Rfilt->cols, CV_32FC1);
    CvMat *Rfilt_3 = cvCreateMat(Rfilt->rows, Rfilt->cols, CV_32FC1);
    cvSplit(Rfilt, Rfilt_1, Rfilt_2, Rfilt_3, NULL);
    cvSmooth(Rfilt_1, Rfilt_1, CV_MEDIAN);
    cvSmooth(Rfilt_2, Rfilt_2, CV_MEDIAN);
    cvSmooth(Rfilt_3, Rfilt_3, CV_MEDIAN);
    cvMerge(Rfilt_1, Rfilt_2, Rfilt_3, NULL, Rfilt);
    cvReleaseMat(&Rfilt_1);
    cvReleaseMat(&Rfilt_2);
    cvReleaseMat(&Rfilt_3);

    cvResize(Rfilt, R, CV_INTER_CUBIC);
    outptr = (uchar *)out->imageData;
    double minR = 256;
    for (int i = 0; i < height; i++) {
        if (colorSpace == ACE_COLORSPACE_GRAY) {
            for (int j = 0; j < width; j++) {
                rptr[j] = (((double)outptr[j] + addFac) / fac) * rptr[j] - addResp;
                if (rptr[j] < minR)  minR = rptr[j];
                if (rptr[j] > maxR)  maxR = rptr[j];
            }
            rptr += width;
            outptr += outstep;
        } else {
            for (int j = 0; j < 3 * width; j += 3) {
                if ((channels & ACE_CHANNEL_1) > 0) {
                    rptr[j + 0] = (((double)outptr[j + 0] + addFac) / fac) * rptr[j + 0] - addResp;
                    if (fabs(rptr[j + 0]) > 1) {
                        if (saturation == ACE_SATURATE) {
                            rptr[j + 0] /= fabs(rptr[j + 0]);
                        } else if (saturation == ACE_REEVALUATE) {
                            rptr[j + 0] = response(out, values, distances, i, j / 3, 0);
                        }
                    }
                    if (fabs(rptr[j + 0]) > maxR) {
                        maxR = fabs(rptr[j + 0]);
                    }
                }
                if ((channels & ACE_CHANNEL_2) > 0) {
                    rptr[j + 1] = (((double)outptr[j + 1] + addFac) / fac) * rptr[j + 1] - addResp;
                    if (fabs(rptr[j + 1]) > 1) {
                        if (saturation == ACE_SATURATE) {
                            rptr[j + 1] /= fabs(rptr[j + 1]);
                        } else if (saturation == ACE_REEVALUATE) {
                            rptr[j + 1] = response(out, values, distances, i, j / 3, 1);
                        }
                    }
                    if (fabs(rptr[j + 1]) > maxR) {
                        maxR = fabs(rptr[j + 1]);
                    }
                }
                if ((channels & ACE_CHANNEL_3) > 0) {
                    rptr[j + 2] = (((double)outptr[j + 2] + addFac) / fac) * rptr[j + 2] - addResp;
                    if (fabs(rptr[j + 2]) > 1) {
                        if (saturation == ACE_SATURATE) {
                            rptr[j + 2] /= fabs(rptr[j + 2]);
                        } else if (saturation == ACE_REEVALUATE) {
                            rptr[j + 2] = response(out, values, distances, i, j / 3, 2);
                        }
                    }
                    if (fabs(rptr[j + 2]) > maxR) {
                        maxR = fabs(rptr[j + 2]);
                    }
                }
            }
            rptr += 3 * width;
            outptr += outstep;
        }
    }

    double sc = maxR / 127.5;
    rptr = R->data.fl;
    outptr = (uchar *)out->imageData;
    for (int i = 0; i < height; i++) {
        if (colorSpace == ACE_COLORSPACE_GRAY) {
            for(int j = 0; j < width; j++) {
//                outptr[j] = (uchar)(255.0 * (rptr[j] - minR) / (maxR - minR));
                outptr[j] = (uchar)(127.5 + rptr[j] / sc);
            }
            rptr += width;
            outptr += outstep;
        } else {
            for(int j = 0; j < 3 * width; j += 3) {
                if ((channels & ACE_CHANNEL_1) > 0) {
                    outptr[j + 0] = (uchar)(127.5 + rptr[j + 0] / sc);
                }
                if ((channels & ACE_CHANNEL_2) > 0) {
                    outptr[j + 1] = (uchar)(127.5 + rptr[j + 1] / sc);
                }
                if ((channels & ACE_CHANNEL_3) > 0) {
                    outptr[j + 2] = (uchar)(127.5 + rptr[j + 2] / sc);
                }
            }
            rptr += 3 * width;
            outptr += outstep;
        }
    }

    cvReleaseMat(&R);
    cvReleaseMat(&Rfilt);

    switch (colorSpace) {
    case ACE_COLORSPACE_HSV:
        cvCvtColor(out, out, CV_HLS2RGB);
        break;
    case ACE_COLORSPACE_YCrCb:
        cvCvtColor(out, out, CV_YCrCb2RGB);
        break;
    }
}

double ACE::response(IplImage *I, CvMat* values, CvMat* distances, int x, int y, int channel)
{
    // ACE response for a specific channel sampled at random Gauss values.
    int step = I->widthStep;
    uchar *iptr = (uchar *)I->imageData;
    float *valptr = values->data.fl;
    float *distptr = distances->data.fl;
    float r_sum = 0.0;
    float r_dist = 0.000001;
    float val = 0.0;
    if (colorSpace == ACE_COLORSPACE_GRAY) {
        val = (float)iptr[x * step + y];
    } else {
        val = (float)iptr[x * step + 3 * y + channel];
    }
    for (int i = 0; i < values->width; i++) {
        int px = x + (int)valptr[i];
        int py = y + (int)valptr[values->width + i];
        if (px >= 0 && px < I->height && py >= 0 && py < I->width && x != px && y != py) {
            if (colorSpace == ACE_COLORSPACE_GRAY) {
                r_sum += resp(val - (float)iptr[px * step + py]) / distptr[i];
            } else {
                r_sum += resp(val - (float)iptr[px * step + 3*py + channel]) / distptr[i];
            }
            r_dist += 255.0 / distptr[i];
        }
    }
    return r_sum / r_dist;
}

double ACE::resp(double x)
{
    // Linear saturation response with slope 5.
    x *= 2;
    x = std::max(-255.0, x);
    x = std::min(x, 255.0);
    return x;
}
