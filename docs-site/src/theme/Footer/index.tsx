import type {ReactNode} from 'react';
import Link from '@docusaurus/Link';

const DOCS = [
  {label: 'Getting started', to: '/docs/getting-started/introduction'},
  {label: 'Architecture', to: '/docs/architecture/impact-detector'},
  {label: 'API reference', to: '/docs/api/impact-detector'},
];

const PROJECT = [
  {label: 'GitHub', href: 'https://github.com/Tropfan32/als-by-absolute'},
  {label: 'Pedro Pathing', href: 'https://pedropathing.com'},
];

const CONTACT = [
  {label: 'Instagram', href: 'https://www.instagram.com/ftc_sunrise'},
  {label: 'Cooperate', href: 'mailto:kairatnariman0001@gmail.com'},
  {label: 'News', href: 'https://t.me/sunriseeeetg'},
];

function Item({
  label,
  to,
  href,
}: {
  label: string;
  to?: string;
  href?: string;
}): ReactNode {
  if (href) {
    return (
      <li className="footer__item">
        <a
          className="footer__link-item"
          href={href}
          target={href.startsWith('mailto:') ? undefined : '_blank'}
          rel={href.startsWith('mailto:') ? undefined : 'noopener noreferrer'}>
          {label}
        </a>
      </li>
    );
  }
  return (
    <li className="footer__item">
      <Link className="footer__link-item" to={to}>
        {label}
      </Link>
    </li>
  );
}

export default function Footer(): ReactNode {
  return (
    <footer className="footer footer--dark">
      <div className="container container-fluid">
        <div className="row footer__links">
          <div className="col footer__col">
            <div className="footer__title">Docs</div>
            <ul className="footer__items clean-list">
              {DOCS.map((item) => (
                <Item key={item.label} {...item} />
              ))}
            </ul>
          </div>
          <div className="col footer__col">
            <div className="footer__title">Project</div>
            <ul className="footer__items clean-list">
              {PROJECT.map((item) => (
                <Item key={item.label} {...item} />
              ))}
            </ul>
          </div>
          <div className="col footer__col">
            <div className="footer__title">Contact</div>
            <ul className="footer__items clean-list">
              {CONTACT.map((item) => (
                <Item key={item.label} {...item} />
              ))}
            </ul>
          </div>
        </div>
        <div className="footer__bottom text--center">
          <div className="footer__copyright">
            © {new Date().getFullYear()} AACS. All rights reserved.
          </div>
        </div>
      </div>
    </footer>
  );
}
