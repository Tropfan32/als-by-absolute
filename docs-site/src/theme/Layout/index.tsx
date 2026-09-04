import type {ReactNode} from 'react';
import Layout from '@theme-original/Layout';
import Head from '@docusaurus/Head';

export default function LayoutWrapper(props: {
  children: ReactNode;
  [key: string]: unknown;
}): ReactNode {
  return (
    <>
      <Layout {...props} />
      <Head>
        <title>aacs</title>
      </Head>
    </>
  );
}
