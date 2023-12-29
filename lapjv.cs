using System;
using System.Runtime.InteropServices;

namespace ByteTrackCSharp;

internal class lapjv
{
    [DllImport("msvcrt.dll",
        EntryPoint = "memset",
        CallingConvention = CallingConvention.Cdecl,
        SetLastError = false)]
    public static extern IntPtr MemSet(IntPtr dest, int c, int count);

    const int LARGE = 1000000;

    private static unsafe bool TryLapjv_Cpp_New<T>(ref T* ptr, int count) where T : unmanaged
    {
        ptr = (T*)Marshal.AllocHGlobal(sizeof(T) * count);
        if (ptr == null)
            return false;
        return true;
    }

    private static unsafe void Lapjv_Cpp_Free<T>(ref T* ptr) where T : unmanaged
    {
        if (ptr != null)
        {
            Marshal.FreeHGlobal((nint)ptr);
            ptr = null;
        }
    }

    private static unsafe void Lapjv_Cpp_Swap_Indices(ref int a, ref int b)
    {
        (a, b) = (b, a);
    }

    /** Column-reduction and reduction transfer for a dense cost matrix.
*/
    public static unsafe int _ccrrt_dense(int n, double** cost,
        int* free_rows, int* x, int* y, double* v)
    {
        int n_free_rows;
        bool* unique = null;

        for (int i = 0; i < n; i++)
        {
            x[i] = -1;
            v[i] = LARGE;
            y[i] = 0;
        }
        for (int i = 0; i < n; i++)
        {
            for (int j = 0; j < n; j++)
            {
                double c = cost[i][j];
                if (c < v[j])
                {
                    v[j] = c;
                    y[j] = i;
                }
            }
        }
        if (TryLapjv_Cpp_New(ref unique, n) == false)
            return -1;


        MemSet((nint)unique, 1, n);
        {
            int j = n;
            do
            {
                j--;
                int i = y[j];
                if (x[i] < 0)
                {
                    x[i] = j;
                }
                else
                {
                    unique[i] = false;
                    y[j] = -1;
                }
            } while (j > 0);
        }
        n_free_rows = 0;
        for (int i = 0; i < n; i++)
        {
            if (x[i] < 0)
            {
                free_rows[n_free_rows++] = i;
            }
            else if (unique[i])
            {
                int j = x[i];
                double min = LARGE;
                for (int j2 = 0; j2 < n; j2++)
                {
                    if (j2 == (int)j)
                    {
                        continue;
                    }
                    double c = cost[i][j2] - v[j2];
                    if (c < min)
                    {
                        min = c;
                    }
                }
                v[j] -= min;
            }
        }
        Lapjv_Cpp_Free(ref unique);
        return n_free_rows;
    }


    /** Augmenting row reduction for a dense cost matrix.
         */
    public static unsafe int _carr_dense(
        int n, double** cost, int n_free_rows,
        int* free_rows, int* x, int* y, double* v)
    {
        int current = 0;
        int new_free_rows = 0;
        int rr_cnt = 0;
        while (current < n_free_rows)
        {
            int i0;
            int j1, j2;
            double v1, v2, v1_new;
            bool v1_lowers;

            rr_cnt++;
            int free_i = free_rows[current++];
            j1 = 0;
            v1 = cost[free_i][0] - v[0];
            j2 = -1;
            v2 = LARGE;
            for (int j = 1; j < n; j++)
            {
                double c = cost[free_i][j] - v[j];
                if (c < v2)
                {
                    if (c >= v1)
                    {
                        v2 = c;
                        j2 = j;
                    }
                    else
                    {
                        v2 = v1;
                        v1 = c;
                        j2 = j1;
                        j1 = j;
                    }
                }
            }
            i0 = y[j1];
            v1_new = v[j1] - (v2 - v1);
            v1_lowers = v1_new < v[j1];
            if (rr_cnt < current * n)
            {
                if (v1_lowers)
                {
                    v[j1] = v1_new;
                }
                else if (i0 >= 0 && j2 >= 0)
                {
                    j1 = j2;
                    i0 = y[j2];
                }
                if (i0 >= 0)
                {
                    if (v1_lowers)
                    {
                        free_rows[--current] = i0;
                    }
                    else
                    {
                        free_rows[new_free_rows++] = i0;
                    }
                }
            }
            else
            {
                if (i0 >= 0)
                {
                    free_rows[new_free_rows++] = i0;
                }
            }
            x[free_i] = j1;
            y[j1] = free_i;
        }
        return new_free_rows;
    }


    /** Find columns with minimum d[j] and put them on the SCAN list.
         */
    public static unsafe int _find_dense(int n, int lo, double* d, int* cols, int* y)
    {
        int hi = lo + 1;
        double mind = d[cols[lo]];
        for (int k = hi; k < n; k++)
        {
            int j = cols[k];
            if (d[j] <= mind)
            {
                if (d[j] < mind)
                {
                    hi = lo;
                    mind = d[j];
                }
                cols[k] = cols[hi];
                cols[hi++] = j;
            }
        }
        return hi;
    }


    // Scan all columns in TODO starting from arbitrary column in SCAN
    // and try to decrease d of the TODO columns using the SCAN column.
    public static unsafe int _scan_dense(int n, double** cost,
        int* plo, int* phi, double* d, int* cols, int* pred, int* y, double* v)
    {
        int lo = *plo;
        int hi = *phi;
        double h, cred_ij;

        while (lo != hi)
        {
            int j = cols[lo++];
            int i = y[j];
            double mind = d[j];
            h = cost[i][j] - v[j] - mind;
            // For all columns in TODO
            for (int k = hi; k < n; k++)
            {
                j = cols[k];
                cred_ij = cost[i][j] - v[j] - h;
                if (cred_ij < d[j])
                {
                    d[j] = cred_ij;
                    pred[j] = i;
                    if (cred_ij == mind)
                    {
                        if (y[j] < 0)
                        {
                            return j;
                        }
                        cols[k] = cols[hi];
                        cols[hi++] = j;
                    }
                }
            }
        }
        *plo = lo;
        *phi = hi;
        return -1;
    }


    /** Single iteration of modified Dijkstra shortest path algorithm as explained in the JV paper.
         *
         * This is a dense matrix version.
         *
         * \return The closest free column index.
         */
    public static unsafe int find_path_dense(int n, double** cost, int start_i, int* y, double* v, int* pred)
    {
        int lo = 0, hi = 0;
        int final_j = -1;
        int n_ready = 0;
        int* cols = null;
        double* d = null;

        if (TryLapjv_Cpp_New(ref cols, n) == false)
            return -1;

        if (TryLapjv_Cpp_New(ref d, n) == false)
            return -1;

        for (int i = 0; i < n; i++)
        {
            cols[i] = i;
            pred[i] = start_i;
            d[i] = cost[start_i][i] - v[i];
        }

        while (final_j == -1)
        {
            // No columns left on the SCAN list.
            if (lo == hi)
            {
                n_ready = lo;
                hi = _find_dense(n, lo, d, cols, y);
                for (int k = lo; k < hi; k++)
                {
                    int j = cols[k];
                    if (y[j] < 0)
                    {
                        final_j = j;
                    }
                }
            }
            if (final_j == -1)
            {
                final_j = _scan_dense(
                    n, cost, &lo, &hi, d, cols, pred, y, v);
            }
        }

        {
            double mind = d[cols[lo]];
            for (int k = 0; k < n_ready; k++)
            {
                int j = cols[k];
                v[j] += d[j] - mind;
            }
        }

        Lapjv_Cpp_Free(ref cols);
        Lapjv_Cpp_Free(ref d);

        return final_j;
    }


    /** Augment for a dense cost matrix.
         */
    public static unsafe int _ca_dense(
        int n, double** cost,
        int n_free_rows,
        int* free_rows, int* x, int* y, double* v)
    {
        int* pred = null;

        if (TryLapjv_Cpp_New(ref pred, n) == false)
            return -1;

        for (int* pfree_i = free_rows; pfree_i < free_rows + n_free_rows; pfree_i++)
        {
            int i = -1, j;
            int k = 0;

            j = find_path_dense(n, cost, *pfree_i, y, v, pred);
            if (j < 0)
            {
                throw new Exception("Error occured in _ca_dense(): j < 0");
            }
            if (j >= n)
            {
                throw new Exception("Error occured in _ca_dense(): j >= n");
            }
            while (i != *pfree_i)
            {
                i = pred[j];
                y[j] = i;
                Lapjv_Cpp_Swap_Indices(ref j, ref x[i]);
                k++;
                if (k >= n)
                {
                    throw new Exception("Error occured in _ca_dense(): k >= n");
                }
            }
        }
        Lapjv_Cpp_Free(ref pred);
        return 0;
    }
    public static unsafe int lapjv_internal(int n, double** cost, int* x, int* y)
    {
        int ret;
        int* free_rows = null;
        double* v = null;

        if (TryLapjv_Cpp_New(ref free_rows, n) == false)
            return -1;
        if (TryLapjv_Cpp_New(ref v, n) == false)
            return -1;

        ret = _ccrrt_dense(n, cost, free_rows, x, y, v);
        int i = 0;
        while (ret > 0 && i < 2)
        {
            ret = _carr_dense(n, cost, ret, free_rows, x, y, v);
            i++;
        }
        if (ret > 0)
        {
            ret = _ca_dense(n, cost, ret, free_rows, x, y, v);
        }

        Lapjv_Cpp_Free(ref v);
        Lapjv_Cpp_Free(ref free_rows);
        return ret;
    }
}